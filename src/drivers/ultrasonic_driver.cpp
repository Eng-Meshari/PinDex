#include "ultrasonic_driver.h"

#include <Arduino.h>
#include <stdio.h>

#include "../../config.h"

// One measurement (TriggerPreparation, then WaitingForEcho):
//   1. Pending until ULTRASONIC_MIN_INTERVAL_MS after the previous trigger
//      (the engine has already waited the power stabilization);
//   2. ECHO must read LOW, else "echo-stuck-high" and no trigger is sent;
//   3. the capture is cleared and armed; TRIG is already LOW (latched LOW
//      when the pins were bound, and after every pulse);
//   4. TRIG HIGH for >= ULTRASONIC_TRIGGER_PULSE_US (a bounded ~10 us busy
//      wait, no critical section), then LOW. The trigger reference is
//      taken just before TRIG goes LOW;
//   5. the ISR timestamps the first rising edge and the falling edge after
//      it, nothing else;
//   6. Pending until the capture completes. The falling edge must lie
//      within ULTRASONIC_ECHO_DEADLINE_US of the reference (one budget for
//      both edges), judged from its own timestamp however late the next
//      call comes.
// Edges belong to the transaction armed in step 3: anything stamped before
// the reference (trigger coupling, a pending stale edge) is rejected, and
// nothing is recorded while disarmed. probe() needs
// ULTRASONIC_CONFIRM_SAMPLES consecutive fresh accepted echoes; readSample()
// takes one fresh echo. A result is never reused.
//
// ESP32 core 3.x: pinMode() keeps a pin's interrupt type, so only
// detachInterrupt() stops the ECHO interrupt; release() always detaches.

static_assert(ULTRASONIC_TRIGGER_PULSE_US >= 10, "the HC-SR04 needs a >= 10 us trigger pulse");
static_assert(ULTRASONIC_ECHO_DEADLINE_US < ULTRASONIC_MIN_INTERVAL_MS * 1000UL,
              "a transaction must end before the next trigger is allowed");
static_assert(ULTRASONIC_MIN_DISTANCE_CM > 0.0f && ULTRASONIC_MIN_DISTANCE_CM < ULTRASONIC_MAX_DISTANCE_CM,
              "empty distance range");
static_assert(ULTRASONIC_MAX_DISTANCE_CM * ULTRASONIC_US_PER_CM < ULTRASONIC_ECHO_DEADLINE_US,
              "the deadline must cover the longest accepted echo");
static_assert(ULTRASONIC_CONFIRM_SAMPLES >= 1, "probe() needs at least one echo");
static_assert(ULTRASONIC_SAMPLE_INTERVAL_MS >= ULTRASONIC_MIN_INTERVAL_MS,
              "monitoring would violate the trigger spacing");
static_assert(ULTRASONIC_OPERATION_TIMEOUT_MS * 1000UL >
                  (ULTRASONIC_CONFIRM_SAMPLES - 1) * ULTRASONIC_MIN_INTERVAL_MS * 1000UL +
                      ULTRASONIC_ECHO_DEADLINE_US,
              "the engine timeout must cover a whole probe()");

namespace {

const DriverRequirements kRequirements = {
    ProbeTopology::FourWirePowered,     // VCC / TRIG / ECHO / GND
    ExcitationPolicy::ProtectedActive,  // engine switches the backend supply
    ULTRASONIC_POWER_STABILIZE_MS,      // powerStabilizeMs
    ULTRASONIC_MIN_INTERVAL_MS,         // minAcquisitionIntervalMs
    ULTRASONIC_SAMPLE_INTERVAL_MS,      // sampleIntervalMs
    ULTRASONIC_OPERATION_TIMEOUT_MS,    // operationTimeoutMs
    kBackendSupply5V,                   // on top of the four-wire baseline
    {"TRG", "ECH"},                     // signalLabel: SIG1, SIG2
};

// Guards edge_/riseUs_/fallUs_ between the ISR and the driver. Held only
// for a few field copies, never across a wait.
portMUX_TYPE echoMux = portMUX_INITIALIZER_UNLOCKED;

}  // namespace

const char *ultrasonicCheckEcho(uint32_t triggerUs, uint32_t riseUs, uint32_t fallUs, uint32_t &echoUs,
                                float &distanceCm) {
  if (static_cast<int32_t>(riseUs - triggerUs) < 0) return "pre-trigger-echo";
  if (static_cast<int32_t>(fallUs - riseUs) < 0) return "malformed-echo";
  if (fallUs - triggerUs > ULTRASONIC_ECHO_DEADLINE_US) return "echo-timeout";
  const uint32_t widthUs = fallUs - riseUs;
  const float cm = static_cast<float>(widthUs) / ULTRASONIC_US_PER_CM;
  if (cm < ULTRASONIC_MIN_DISTANCE_CM || cm > ULTRASONIC_MAX_DISTANCE_CM) return "out-of-range";
  echoUs = widthUs;
  distanceCm = cm;
  return nullptr;
}

const char *UltrasonicDriver::getName() const { return "HC-SR04 Ultrasonic"; }

DriverOperationStatus UltrasonicDriver::getOperationStatus() const { return status_; }

const DriverRequirements &UltrasonicDriver::getRequirements() const { return kRequirements; }

const char *UltrasonicDriver::getFailureReason() const { return failureReason_; }

bool UltrasonicDriver::probe(const ProbeContext &context) {
  if (context.topology != ProbeTopology::FourWirePowered) {
    return fail("topology-mismatch");  // never treat other roles as TRIG/ECHO
  }
  const FourWirePins &pins = context.fourWire;
  if (status_ == DriverOperationStatus::Pending) {
    if (operation_ != Operation::Probe || pins.vcc != pins_.vcc || pins.sig1 != pins_.sig1 ||
        pins.sig2 != pins_.sig2 || pins.gnd != pins_.gnd) {
      return fail("contract-violation");
    }
  } else {
    release();  // a new binding: old pins released, confirmation restarts
    pins_ = pins;
    bound_ = false;
    accepted_ = 0;
    begin(Operation::Probe);
  }
  if (!advance()) return false;
  bound_ = true;
  return true;
}

bool UltrasonicDriver::readSample(char *outLine1, char *outLine2, size_t maxLen) {
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

  const unsigned long tenths = static_cast<unsigned long>(distanceCm_ * 10.0f + 0.5f);
  snprintf(outLine1, maxLen, "Dist: %lu.%lu cm", tenths / 10, tenths % 10);
  snprintf(outLine2, maxLen, "Echo: %lu us", static_cast<unsigned long>(echoUs_));
  return true;
}

void UltrasonicDriver::reset() {
  release();
  operation_ = Operation::None;
  status_ = DriverOperationStatus::Idle;
  step_ = Step::Idle;
  failureReason_ = "";
  pins_ = FourWirePins{};
  bound_ = false;
  accepted_ = 0;
  triggered_ = false;
  triggerUs_ = 0;
  echoUs_ = 0;
  distanceCm_ = 0.0f;
}

void UltrasonicDriver::begin(Operation operation) {
  operation_ = operation;
  status_ = DriverOperationStatus::Pending;
  failureReason_ = "";
  step_ = Step::TriggerPreparation;
  if (!wired_) wire();
}

// One step of the current operation; true once it has succeeded.
bool UltrasonicDriver::advance() {
  if (step_ == Step::TriggerPreparation) {
    if (triggered_ && micros() - triggerUs_ < ULTRASONIC_MIN_INTERVAL_MS * 1000UL) return false;  // spacing
    if (digitalRead(pins_.sig2) != LOW) return fail("echo-stuck-high");
    trigger();
    return false;  // an echo is at least ~100 us away
  }

  const uint32_t nowUs = micros();  // first: an edge missing below came after it
  portENTER_CRITICAL(&echoMux);
  const uint8_t edge = edge_;
  const uint32_t riseUs = riseUs_;
  const uint32_t fallUs = fallUs_;
  portEXIT_CRITICAL(&echoMux);

  if (edge == kMalformed) return fail("malformed-echo");
  if (edge != kComplete) {
    if (nowUs - triggerUs_ <= ULTRASONIC_ECHO_DEADLINE_US) return false;  // still due
    return fail(edge == kAwaitRise ? "no-echo" : "echo-timeout");
  }

  setEdge(kDisarmed);
  uint32_t echoUs = 0;
  float distanceCm = 0.0f;
  if (const char *rejected = ultrasonicCheckEcho(triggerUs_, riseUs, fallUs, echoUs, distanceCm)) {
    return fail(rejected);
  }
  echoUs_ = echoUs;
  distanceCm_ = distanceCm;
  if (operation_ == Operation::Probe && ++accepted_ < ULTRASONIC_CONFIRM_SAMPLES) {
    step_ = Step::TriggerPreparation;  // the next fresh echo, after the spacing
    return false;
  }
  operation_ = Operation::None;
  status_ = DriverOperationStatus::Succeeded;
  step_ = Step::Idle;
  return true;
}

void UltrasonicDriver::trigger() {
  portENTER_CRITICAL(&echoMux);
  riseUs_ = 0;
  fallUs_ = 0;
  edge_ = kAwaitRise;
  portEXIT_CRITICAL(&echoMux);

  digitalWrite(pins_.sig1, HIGH);
  const uint32_t startUs = micros();
  while (micros() - startUs < ULTRASONIC_TRIGGER_PULSE_US) {
  }
  triggerUs_ = micros();
  digitalWrite(pins_.sig1, LOW);

  triggered_ = true;
  step_ = Step::WaitingForEcho;
}

void ARDUINO_ISR_ATTR UltrasonicDriver::onEchoEdge(void *self) {
  static_cast<UltrasonicDriver *>(self)->captureEdge();
}

// ISR: one timestamp and a state change. No logging, formatting or
// allocation. A level that does not match the awaited edge (a glitch or a
// missed edge) marks the transaction malformed.
void ARDUINO_ISR_ATTR UltrasonicDriver::captureEdge() {
  const uint32_t nowUs = micros();
  const int level = digitalRead(pins_.sig2);
  portENTER_CRITICAL_ISR(&echoMux);
  if (edge_ == kAwaitRise) {
    if (level == HIGH) {
      riseUs_ = nowUs;
      edge_ = kAwaitFall;
    } else {
      edge_ = kMalformed;
    }
  } else if (edge_ == kAwaitFall) {
    if (level == LOW) {
      fallUs_ = nowUs;
      edge_ = kComplete;
    } else {
      edge_ = kMalformed;
    }
  }  // disarmed, complete or malformed: ignored
  portEXIT_CRITICAL_ISR(&echoMux);
}

void UltrasonicDriver::setEdge(Edge edge) {
  portENTER_CRITICAL(&echoMux);
  edge_ = edge;
  portEXIT_CRITICAL(&echoMux);
}

// TRIG latched LOW before its driver turns on (core 3.x), so it never
// glitches HIGH; ECHO a plain input with the capture disarmed.
void UltrasonicDriver::wire() {
  setEdge(kDisarmed);
  digitalWrite(pins_.sig1, LOW);
  pinMode(pins_.sig1, OUTPUT);
  pinMode(pins_.sig2, INPUT);
  attachInterruptArg(pins_.sig2, onEchoEdge, this, CHANGE);
  wired_ = true;
}

// Only makes pins safer: capture disarmed and its interrupt detached, so no
// late edge can reach a later transaction, then TRIG back to plain INPUT.
// Unwired pins are not touched.
void UltrasonicDriver::release() {
  setEdge(kDisarmed);
  if (!wired_) return;
  detachInterrupt(pins_.sig2);
  pinMode(pins_.sig1, INPUT);
  wired_ = false;
}

bool UltrasonicDriver::fail(const char *reason) {
  release();
  operation_ = Operation::None;
  status_ = DriverOperationStatus::Failed;
  step_ = Step::Idle;
  failureReason_ = reason;
  accepted_ = 0;
  return false;
}
