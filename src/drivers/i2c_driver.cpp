#include "i2c_driver.h"

#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include "../../config.h"
#include "i2c_bus.h"

// probe(), after the engine's power stabilization:
//   1. start the probe controller on SDA (SIG1) / SCL (SIG2);
//   2. a sweep: SDA and SCL must read released-HIGH (a LOW line gets
//      I2C_IDLE_SETTLE_MS), then one address-only write with STOP to the
//      next address of 0x08-0x77, one per call, Pending in between; the
//      whole sweep must finish within I2C_SWEEP_DEADLINE_MS;
//   3. an ACK is recorded, a NACK skipped; a timeout or bus error ends the
//      attempt at once (no further transactions on a faulted bus);
//   4. after the last address and idle lines: no ACK, or every address
//      ACKing, is rejected; otherwise another full sweep runs, and two
//      consecutive sweeps with the same address set confirm (at most
//      I2C_MAX_SWEEPS sweeps, else "unstable-addresses").
// readSample(): one address-only write to the next confirmed address
// (round-robin), accepted once the lines are idle again. It is a liveness
// check, not a sensor reading; it rotates the presented address.
// Teardown (fail(), reset()) stops issuing transactions and stops the
// controller (pulls off, plain INPUT) before the engine removes power.
// Nothing is ever written beyond the address byte, but an address-only
// write is not guaranteed to be side-effect-free for every device.

static_assert(I2C_FIRST_ADDRESS >= 0x08 && I2C_LAST_ADDRESS <= 0x77 && I2C_FIRST_ADDRESS <= I2C_LAST_ADDRESS,
              "only the non-reserved 7-bit addresses 0x08-0x77 may be scanned");
static_assert(I2C_MAX_SWEEPS >= 2, "confirmation needs two independent sweeps");
static_assert(I2C_IDLE_SETTLE_MS >= 1 && I2C_TRANSACTION_TIMEOUT_MS >= 1, "bounds must be non-zero");
static_assert(I2C_SWEEP_DEADLINE_MS > I2C_LAST_ADDRESS - I2C_FIRST_ADDRESS + 1,
              "a sweep at one address per millisecond must fit the deadline");
static_assert(I2C_OPERATION_TIMEOUT_MS > I2C_MAX_SWEEPS * I2C_SWEEP_DEADLINE_MS + I2C_IDLE_SETTLE_MS,
              "the engine timeout must cover every sweep of a probe()");

namespace {

constexpr uint8_t kAddressCount = I2C_LAST_ADDRESS - I2C_FIRST_ADDRESS + 1;

const DriverRequirements kRequirements = {
    ProbeTopology::FourWirePowered,     // VCC / SDA / SCL / GND
    ExcitationPolicy::ProtectedActive,  // the engine switches the supply
    I2C_POWER_STABILIZE_MS,             // powerStabilizeMs
    0,                                  // minAcquisitionIntervalMs: no protocol spacing
    I2C_SAMPLE_INTERVAL_MS,             // sampleIntervalMs
    I2C_OPERATION_TIMEOUT_MS,           // operationTimeoutMs
    SupplyProfile::Supply3V3,           // 3.3 V backend; else only the experimental GPIO profile
    true,                               // openDrainSignals: SDA/SCL are only released or sunk
    {"SDA", "SCL"},                     // signalLabel: SIG1, SIG2
    false,                              // dataIsOutput: three-wire only
};

struct Hint {
  uint8_t first;
  uint8_t last;
  const char *text;
};

// Common candidates only: not exhaustive, not exclusive, never proof. A
// PCF8574-family answer does not prove an LCD is attached.
const Hint kHints[] = {
    {0x23, 0x23, "BH1750?"},
    {0x27, 0x27, "PCF8574 / LCD?"},
    {0x3C, 0x3D, "SSD1306 / OLED?"},
    {0x3F, 0x3F, "PCF8574A / LCD?"},
    {0x48, 0x48, "ADS1115 / Other?"},
    {0x50, 0x57, "AT24Cxx / EEPROM?"},
    {0x68, 0x68, "MPU6050 / DS3231?"},
    {0x69, 0x69, "MPU6050 / IMU?"},
    {0x76, 0x77, "BMP280 / BME280?"},
};

bool has(const I2cAddressSet &s, uint8_t address) {
  return address < 128 && ((s.bits[address >> 5] >> (address & 31)) & 1u) != 0;
}

void add(I2cAddressSet &s, uint8_t address) {
  if (address >= 128 || has(s, address)) return;
  s.bits[address >> 5] |= 1u << (address & 31);
  s.count++;
}

bool sameSet(const I2cAddressSet &a, const I2cAddressSet &b) {
  return a.count == b.count && memcmp(a.bits, b.bits, sizeof a.bits) == 0;
}

// The next address of the set after `after`, wrapping; 0 if it is empty.
uint8_t nextIn(const I2cAddressSet &s, uint8_t after) {
  for (unsigned a = after + 1u; a <= I2C_LAST_ADDRESS; a++) {
    if (has(s, static_cast<uint8_t>(a))) return static_cast<uint8_t>(a);
  }
  for (unsigned a = I2C_FIRST_ADDRESS; a <= after && a <= I2C_LAST_ADDRESS; a++) {
    if (has(s, static_cast<uint8_t>(a))) return static_cast<uint8_t>(a);
  }
  return 0;
}

// 1-based position of `address` in the set.
unsigned positionIn(const I2cAddressSet &s, uint8_t address) {
  unsigned position = 0;
  for (unsigned a = I2C_FIRST_ADDRESS; a <= address; a++) position += has(s, static_cast<uint8_t>(a)) ? 1 : 0;
  return position;
}

}  // namespace

const char *i2cAddressHint(uint8_t address) {
  for (const Hint &h : kHints) {
    if (address >= h.first && address <= h.last) return h.text;
  }
  return "Unknown model";
}

const char *I2cDriver::getName() const { return "I2C Device"; }

DriverOperationStatus I2cDriver::getOperationStatus() const { return status_; }

const DriverRequirements &I2cDriver::getRequirements() const { return kRequirements; }

const char *I2cDriver::getFailureReason() const { return failureReason_; }

bool I2cDriver::failureIsMeasurementError() const { return measurementError_; }

bool I2cDriver::resourcesReleased() const { return !releaseFailed_; }

bool I2cDriver::probe(const ProbeContext &context) {
  if (context.topology != ProbeTopology::FourWirePowered) {
    return fail("topology-mismatch");  // never treat other roles as SDA/SCL
  }
  const FourWirePins &pins = context.fourWire;
  if (status_ == DriverOperationStatus::Pending) {
    if (operation_ != Operation::Probe || pins.vcc != pins_.vcc || pins.sig1 != pins_.sig1 ||
        pins.sig2 != pins_.sig2 || pins.gnd != pins_.gnd) {
      return fail("contract-violation");
    }
  } else {
    release();  // a new session: any previous one stops first
    pins_ = pins;
    bound_ = false;
    clearResults();  // addresses, hints and confirmation belong to one session
    if (!begin(Operation::Probe)) return false;
    sweeps_ = 0;
    startSweep();
  }
  if (!advance()) return false;
  bound_ = true;
  return true;
}

bool I2cDriver::readSample(char *outLine1, char *outLine2, size_t maxLen) {
  if (outLine1 == nullptr || outLine2 == nullptr || maxLen == 0) {
    return fail("invalid-buffer");  // nothing dereferenced
  }
  outLine1[0] = '\0';
  outLine2[0] = '\0';

  if (status_ == DriverOperationStatus::Pending) {
    if (operation_ != Operation::Sample) return fail("contract-violation");
  } else {
    if (!bound_) return fail("not-probed");
    if (!begin(Operation::Sample)) return false;
    pinging_ = nextIn(confirmed_, shown_);
    step_ = Step::Ping;
  }
  if (!advance()) return false;

  if (confirmed_.count == 1) {
    snprintf(outLine1, maxLen, "Addr: 0x%02X (ACK)", shown_);
  } else {
    snprintf(outLine1, maxLen, "Addr %u/%u: 0x%02X ACK", positionIn(confirmed_, shown_),
             static_cast<unsigned>(confirmed_.count), shown_);
  }
  snprintf(outLine2, maxLen, "%s", i2cAddressHint(shown_));
  return true;
}

void I2cDriver::reset() {
  release();
  operation_ = Operation::None;
  status_ = DriverOperationStatus::Idle;
  step_ = Step::Idle;
  failureReason_ = "";
  measurementError_ = false;
  pins_ = FourWirePins{};
  bound_ = false;
  clearResults();
}

bool I2cDriver::formatDiagnostics(char *buf, size_t len) const {
  if (buf == nullptr || len == 0) return false;
  buf[0] = '\0';
  if (!bound_) return false;
  int used = snprintf(buf, len, "I2C ACK (%u):", static_cast<unsigned>(confirmed_.count));
  for (unsigned a = I2C_FIRST_ADDRESS; a <= I2C_LAST_ADDRESS; a++) {
    if (used < 0 || static_cast<size_t>(used) + 1 >= len) break;  // full: stays terminated
    if (has(confirmed_, static_cast<uint8_t>(a))) used += snprintf(buf + used, len - used, " 0x%02X", a);
  }
  return true;
}

bool I2cDriver::begin(Operation operation) {
  operation_ = operation;
  status_ = DriverOperationStatus::Pending;
  failureReason_ = "";
  measurementError_ = false;
  lineLow_ = false;
  if (busActive_) return true;
  if (releaseFailed_) return fail("bus-release-failed", true);  // never start over an uncertain one
  switch (i2cBusBegin(pins_.sig1, pins_.sig2)) {
    case I2cBusStart::Ok:
      busActive_ = true;
      return true;
    case I2cBusStart::Conflict:
      return fail("bus-conflict", true);  // owned elsewhere: left untouched, never stopped here
    case I2cBusStart::Failed:
      break;
  }
  return fail("bus-init-failed", true);  // the adapter released what it acquired
}

bool I2cDriver::advance() {
  switch (step_) {
    case Step::Sweep: return advanceSweep();
    case Step::Ping:
    case Step::Verify: return advancePing();
    case Step::Idle: break;
  }
  return fail("contract-violation");
}

bool I2cDriver::advanceSweep() {
  if (millis() - sweepStartMs_ >= I2C_SWEEP_DEADLINE_MS) return fail("sweep-deadline");  // rollover-safe
  if (!linesIdle()) return false;
  if (next_ <= I2C_LAST_ADDRESS) {
    const uint8_t address = next_++;
    switch (i2cBusProbe(address)) {
      case I2cBusResult::Ack: add(sweep_, address); break;
      case I2cBusResult::Nack: break;  // an ordinary miss: the sweep goes on
      case I2cBusResult::Timeout: return fail("bus-timeout");
      case I2cBusResult::BusError: return fail("bus-error");
    }
    return false;  // at most one transaction per call
  }

  // Swept, and both lines idle again: judge this sweep on its own.
  if (sweep_.count == 0) return fail("no-ack");
  if (sweep_.count == kAddressCount) return fail("all-addresses-ack");  // not a plausible bus
  sweeps_++;
  if (sweeps_ >= 2 && sameSet(sweep_, previous_)) {
    confirmed_ = sweep_;
    return succeed();
  }
  if (sweeps_ >= I2C_MAX_SWEEPS) return fail("unstable-addresses");
  previous_ = sweep_;
  startSweep();  // a fresh, independent sweep; nothing is reused
  return false;
}

bool I2cDriver::advancePing() {
  if (!linesIdle()) return false;
  if (step_ == Step::Verify) {  // ACKed, and the lines are idle again
    shown_ = pinging_;
    return succeed();
  }
  switch (i2cBusProbe(pinging_)) {
    case I2cBusResult::Ack:
      step_ = Step::Verify;
      return false;
    case I2cBusResult::Nack: return fail("address-lost");
    case I2cBusResult::Timeout: return fail("bus-timeout");
    case I2cBusResult::BusError: break;
  }
  return fail("bus-error");
}

// SDA and SCL must read released-HIGH. A line seen LOW gets
// I2C_IDLE_SETTLE_MS from that moment to release. False = not yet
// (Pending) or failed ("bus-stuck-low").
bool I2cDriver::linesIdle() {
  if (i2cBusIdle()) {
    lineLow_ = false;
    return true;
  }
  const uint32_t now = millis();
  if (!lineLow_) {
    lineLow_ = true;
    lowSinceMs_ = now;
  } else if (now - lowSinceMs_ >= I2C_IDLE_SETTLE_MS) {
    fail("bus-stuck-low");
  }
  return false;
}

void I2cDriver::startSweep() {
  sweep_ = I2cAddressSet{};
  next_ = I2C_FIRST_ADDRESS;
  sweepStartMs_ = millis();
  step_ = Step::Sweep;
}

bool I2cDriver::succeed() {
  operation_ = Operation::None;
  status_ = DriverOperationStatus::Succeeded;
  step_ = Step::Idle;
  return true;
}

bool I2cDriver::fail(const char *reason, bool measurementError) {
  release();
  operation_ = Operation::None;
  status_ = DriverOperationStatus::Failed;
  step_ = Step::Idle;
  failureReason_ = reason;
  measurementError_ = measurementError;
  return false;
}

// Stops issuing transactions (none is ever left pending between calls),
// stops the controller and resets both pads (pulls off, plain INPUT), all
// before the engine removes power. A failed stop keeps releaseFailed_ set
// and is tried once more on each later release(). A controller this driver
// did not start is never stopped.
void I2cDriver::release() {
  if (!busActive_ && !releaseFailed_) return;
  busActive_ = false;
  releaseFailed_ = !i2cBusEnd();
}

void I2cDriver::clearResults() {
  next_ = 0;
  sweeps_ = 0;
  sweep_ = I2cAddressSet{};
  previous_ = I2cAddressSet{};
  confirmed_ = I2cAddressSet{};
  shown_ = 0;
  pinging_ = 0;
  lineLow_ = false;
}
