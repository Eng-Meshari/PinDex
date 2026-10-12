#include "dht11_driver.h"

#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include "../../config.h"

static_assert(DHT_READ_INTERVAL_MS >= DHT_MIN_START_INTERVAL_MS,
              "scheduled reads would violate the start-request spacing");
static_assert(DHT_OPERATION_TIMEOUT_MS > DHT_START_LOW_MS + 7,
              "the timeout must cover the start pulse and the 6.5 ms capture");

namespace {

const DriverRequirements kRequirements = {
    ProbeTopology::ThreeWirePowered,   // VCC / DATA / GND
    ExcitationPolicy::ProtectedActive,  // engine powers VCC/GND
    DHT_POWER_STABILIZE_MS,     // powerStabilizeMs
    DHT_MIN_START_INTERVAL_MS,  // minAcquisitionIntervalMs
    DHT_READ_INTERVAL_MS,       // sampleIntervalMs
    DHT_OPERATION_TIMEOUT_MS,   // operationTimeoutMs
    SupplyProfile::GpioLogic,   // supply: VCC/GND from GPIOs
    true,                       // openDrainSignals: DATA is only pulled up or sunk
    {nullptr, nullptr},         // signalLabel: three-wire roles are VCC/DAT/GND
};

// ---- Bounded DHT response capture ----------------------------------------

constexpr uint32_t kTimedOut = 0xFFFFFFFFUL;

portMUX_TYPE captureMux = portMUX_INITIALIZER_UNLOCKED;

// Masks interrupts on this core for the capture only; the destructor
// restores the previous state on every return path. Never held across the
// start pulse, stabilization, retries, Serial or display work.
struct CaptureCriticalSection {
  CaptureCriticalSection() { portENTER_CRITICAL(&captureMux); }
  ~CaptureCriticalSection() { portEXIT_CRITICAL(&captureMux); }
};

// How long the line stays at `level`, in us. kTimedOut if the phase exceeds
// DHT_PHASE_TIMEOUT_US or the whole response exceeds DHT_FRAME_DEADLINE_US.
uint32_t levelWidthUs(uint8_t pin, int level, uint32_t frameStartUs) {
  const uint32_t startUs = micros();
  for (;;) {
    const uint32_t nowUs = micros();
    if (digitalRead(pin) != level) return nowUs - startUs;
    if (nowUs - startUs > DHT_PHASE_TIMEOUT_US || nowUs - frameStartUs > DHT_FRAME_DEADLINE_US) {
      return kTimedOut;
    }
  }
}

// Releases DATA and records one response. Busy-polls for at most
// DHT_FRAME_DEADLINE_US plus one poll (~4-5 ms for a normal frame) with
// interrupts masked on this core. No Serial, display or allocation here.
DhtError captureFrame(uint8_t pin, DhtPulses &p) {
  memset(&p, 0, sizeof p);
  CaptureCriticalSection lock;

  digitalWrite(pin, HIGH);  // open-drain release; the pull-up raises DATA
  const uint32_t frameStartUs = micros();

  if (levelWidthUs(pin, LOW, frameStartUs) == kTimedOut) return DhtError::StuckLow;
  if (levelWidthUs(pin, HIGH, frameStartUs) == kTimedOut) return DhtError::NoResponse;

  uint32_t w = levelWidthUs(pin, LOW, frameStartUs);
  if (w == kTimedOut) return DhtError::AckTimeout;
  p.ackLowUs = static_cast<uint16_t>(w);
  w = levelWidthUs(pin, HIGH, frameStartUs);
  if (w == kTimedOut) return DhtError::AckTimeout;
  p.ackHighUs = static_cast<uint16_t>(w);

  // Each bit is a LOW phase then a HIGH phase; the last HIGH ends when the
  // sensor pulls low for end-of-frame, so it is also bounded.
  for (uint8_t i = 0; i < DHT_FRAME_BITS; i++) {
    w = levelWidthUs(pin, LOW, frameStartUs);
    if (w == kTimedOut) return DhtError::BitTimeout;
    p.bitLowUs[i] = static_cast<uint16_t>(w);
    w = levelWidthUs(pin, HIGH, frameStartUs);
    if (w == kTimedOut) return DhtError::BitTimeout;
    p.bitHighUs[i] = static_cast<uint16_t>(w);
  }
  return DhtError::None;
}

}  // namespace

const char *Dht11Driver::getName() const { return "DHT11 Temp & Hum"; }

DriverOperationStatus Dht11Driver::getOperationStatus() const { return status_; }

const DriverRequirements &Dht11Driver::getRequirements() const { return kRequirements; }

const char *Dht11Driver::getFailureReason() const { return failureReason_; }

bool Dht11Driver::probe(const ProbeContext &context) {
  if (context.topology != ProbeTopology::ThreeWirePowered) {
    return fail("topology-mismatch");  // never treat passive terminals as VCC/DATA/GND
  }
  const ThreeWirePins &pins = context.powered;
  if (status_ == DriverOperationStatus::Pending) {
    if (operation_ != Operation::Probe || pins.vcc != pinVcc_ || pins.data != pinData_ ||
        pins.gnd != pinGnd_) {
      return fail("contract-violation");
    }
  } else {
    pinVcc_ = pins.vcc;
    pinData_ = pins.data;
    pinGnd_ = pins.gnd;
    bound_ = false;  // re-earned by this probe
    begin(Operation::Probe);
  }
  if (!advance()) return false;
  bound_ = true;
  return true;
}

bool Dht11Driver::readSample(char *outLine1, char *outLine2, size_t maxLen) {
  if (outLine1 == nullptr || outLine2 == nullptr || maxLen == 0) {
    return fail("invalid-buffer");  // nothing dereferenced
  }
  outLine1[0] = '\0';
  outLine2[0] = '\0';

  if (status_ == DriverOperationStatus::Pending) {
    if (operation_ != Operation::Sample) return fail("contract-violation");
  } else {
    if (!bound_) return fail("not-probed");
    begin(Operation::Sample);
  }
  if (!advance()) return false;

  // Integer degrees, as the Phase 2 display showed; line 2 is unused.
  snprintf(outLine1, maxLen, "T: %d*C  |  H: %u%%", reading_.temperatureDeciC / 10,
           static_cast<unsigned>(reading_.humidityPct));
  return true;
}

void Dht11Driver::reset() {
  releaseHeldLine();
  operation_ = Operation::None;
  status_ = DriverOperationStatus::Idle;
  bound_ = false;
  pinVcc_ = pinData_ = pinGnd_ = 0;
  startMs_ = 0;
  memset(&pulses_, 0, sizeof pulses_);
  reading_ = DhtReading{};
  failureReason_ = "";
}

// Starts the 20 ms start pulse; the rest happens in advance().
void Dht11Driver::begin(Operation operation) {
  operation_ = operation;
  status_ = DriverOperationStatus::Pending;
  failureReason_ = "";
  // Open-drain with pull-up: LOW sinks DATA, HIGH only releases it, so DATA
  // is never driven high. digitalWrite() first sets the latch (ESP32 core
  // 3.x) so the driver turns on low, never briefly high.
  digitalWrite(pinData_, LOW);
  pinMode(pinData_, OUTPUT_OPEN_DRAIN | PULLUP);
  holdingLow_ = true;
  startMs_ = millis();
}

// Pending until the start pulse has lasted DHT_START_LOW_MS, then releases
// DATA and captures in the same call so the reply isn't missed.
bool Dht11Driver::advance() {
  if (millis() - startMs_ < DHT_START_LOW_MS) return false;  // still Pending

  DhtError error = captureFrame(pinData_, pulses_);
  holdingLow_ = false;
  pinMode(pinData_, INPUT_PULLUP);  // plain pulled-up input again, outside the lock

  uint8_t bytes[DHT_FRAME_BYTES] = {};
  DhtReading reading{};
  if (error == DhtError::None) error = dhtPulsesToBytes(pulses_, bytes);
  if (error == DhtError::None) error = dhtDecodeFrame(bytes, reading);
  if (error != DhtError::None) return fail(dhtErrorName(error));

  reading_ = reading;
  operation_ = Operation::None;
  status_ = DriverOperationStatus::Succeeded;
  return true;
}

bool Dht11Driver::fail(const char *reason) {
  releaseHeldLine();
  operation_ = Operation::None;
  status_ = DriverOperationStatus::Failed;
  failureReason_ = reason;
  return false;
}

// Only ever makes DATA safer: a start pulse in progress is released to
// plain INPUT (high impedance). Pins this driver isn't sinking are untouched.
void Dht11Driver::releaseHeldLine() {
  if (holdingLow_) pinMode(pinData_, INPUT);
  holdingLow_ = false;
}
