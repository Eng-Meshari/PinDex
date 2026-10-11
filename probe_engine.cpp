#include "probe_engine.h"

#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "sensor_driver.h"
#include "src/drivers/driver_registry.h"

// Safety scope: plain INPUT (high impedance) only means the firmware is not
// driving the probes. It is NOT protection against shorts, overvoltage, ESD
// or charged capacitors. Powered probing drives probe GPIOs as a nominal
// 3.3V logic-level supply, not a regulated rail, and must only be enabled
// behind a reviewed protected interface (see README).
//
// The engine owns power, prechecks, permutations, scheduling, confirmation
// and removal. Drivers (sensor_driver.h) own only DATA-line protocols. This
// file knows no concrete driver: it uses the abstract contract and registry.

static_assert(PROBE_RETRY_COOLDOWN_MS >= PROBE_POWER_OFF_MS,
              "a retry must include the power-off interval");
static_assert(PROBE_RESCAN_INTERVAL_MS >= PROBE_POWER_OFF_MS,
              "a rescan must include the power-off interval");
static_assert(PROBE_CONFIRM_FRAMES >= 1, "confirmation needs at least one acquisition");

const ProbeMapping PROBE_PERMUTATIONS[PROBE_PERMUTATION_COUNT] = {
    // {VCC, DATA, GND} as node indices: 0 = A, 1 = B, 2 = C.
    {0, 1, 2},  // 1: A B C
    {2, 1, 0},  // 2: C B A
    {1, 0, 2},  // 3: B A C
    {1, 2, 0},  // 4: B C A
    {0, 2, 1},  // 5: A C B
    {2, 0, 1},  // 6: C A B
};

namespace {

// Socket labels and GPIOs are kept apart: roles are resolved per node.
const uint8_t kNodeGpio[PROBE_NODE_COUNT] = {PROBE_PIN_A, PROBE_PIN_B, PROBE_PIN_C};
const char kNodeLabel[PROBE_NODE_COUNT] = {'A', 'B', 'C'};

enum class State : uint8_t {
  Disabled,      // powered probing compiled out
  Idle,          // probes INPUT, waiting idleDelayMs before discovery
  Precheck,      // weak-pull diagnostic, one pull-up source node at a time
  PowerUp,       // apply GND, VCC, DATA pull-up for the active mapping
  Stabilizing,   // driver's powerStabilizeMs before the first probe() call
  Probing,       // driver probe() operation in progress
  Confirming,    // candidate powered, waiting for the next acquisition slot
  Monitoring,    // active driver powered, waiting for the next acquisition slot
  Sampling,      // driver readSample() operation in progress
  PowerOffWait,  // discovery: torn down, waiting before the next attempt
  RetryWait,     // saved mapping failed: torn down, cooling down
  Fault,         // precheck rejected the socket; latched, probes INPUT
};

State state = State::Disabled;
uint32_t stateSinceMs = 0;       // entry time of the state (or precheck step)
uint32_t idleDelayMs = 0;
uint32_t lastAcquisitionMs = 0;  // start of the latest probe()/readSample() operation
uint8_t permIndex = 0;           // discovery permutation, 0-based (outer loop)
size_t driverIndex = 0;          // discovery driver, registry order (inner loop)
SensorDriver *savedDriver = nullptr;  // candidate until detected, then active
uint8_t confirmations = 0;       // consecutive successful acquisitions
bool noComponent = false;        // last discovery found nothing
uint8_t precheckSource = 0;
uint8_t precheckSample = 0;
uint8_t precheckFirst[PROBE_NODE_COUNT];
ProbeStatus status;

bool elapsed(uint32_t now, uint32_t since, uint32_t interval) {
  return now - since >= interval;  // unsigned: correct across millis() rollover
}

void enter(State next, uint32_t now) {
  state = next;
  stateSinceMs = now;
}

uint8_t gpio(uint8_t node) { return kNodeGpio[node]; }

const ProbeMapping &activeMapping() {
  return status.hasMapping ? status.mapping : PROBE_PERMUTATIONS[permIndex];
}

// The driver being attempted (discovery) or the saved candidate/active one.
SensorDriver *currentDriver() {
  return savedDriver != nullptr ? savedDriver : sensorDriverAt(driverIndex);
}

// ESP32 core 3.x: digitalWrite() on a pin already set up by pinMode() only
// sets the output latch via gpio_set_level() (see esp32-hal-gpio.c), and
// pinMode() -> ESP-IDF gpio_config() enables the driver without rewriting
// that latch. Writing the level first means the driver turns on at that
// level, never briefly at the opposite one. Confirm with a scope (README).
void driveNode(uint8_t node, uint8_t level) {
  digitalWrite(gpio(node), level);
  pinMode(gpio(node), OUTPUT);
}

void powerUp(const ProbeMapping &m) {
  resetTestPins();
  driveNode(m.gnd, LOW);  // GND first, so VCC never rises without a return
  driveNode(m.vcc, HIGH);
  pinMode(gpio(m.data), INPUT_PULLUP);
}

// DATA first (no pull-up or drive left to back-power the part through its
// DATA pin), then VCC, then every node including GND.
void powerDown(const ProbeMapping &m) {
  pinMode(gpio(m.data), INPUT);
  pinMode(gpio(m.vcc), INPUT);
  resetTestPins();
}

void clearResult() {
  if (savedDriver != nullptr) savedDriver->reset();  // idempotent, never energises
  savedDriver = nullptr;
  status.hasMapping = false;
  status.mapping = ProbeMapping{0, 0, 0};
  status.componentName = nullptr;
  status.detected = false;
  status.readingValid = false;
  status.readingStale = false;
  status.telemetry1[0] = '\0';
  status.telemetry2[0] = '\0';
  status.consecutiveFailures = 0;
  confirmations = 0;
}

void goIdle(uint32_t now, uint32_t delayMs) {
  resetTestPins();
  idleDelayMs = delayMs;
  enter(State::Idle, now);
}

void enterFault(uint32_t now) {
  resetTestPins();
  if (SensorDriver *d = currentDriver()) d->reset();  // cancel anything in flight
  clearResult();  // cancel the scan; nothing is retried until a request
  enter(State::Fault, now);
}

// Precheck step: resetTestPins(), then one node pulled up, the others down.
void applyPrecheckPulls(uint8_t source) {
  resetTestPins();
  for (uint8_t n = 0; n < PROBE_NODE_COUNT; n++) {
    pinMode(gpio(n), n == source ? INPUT_PULLUP : INPUT_PULLDOWN);
  }
}

void beginPrecheck(uint32_t now) {
  precheckSource = 0;
  precheckSample = 0;
  applyPrecheckPulls(precheckSource);
  enter(State::Precheck, now);
}

void startDiscovery(uint32_t now) {
  clearResult();
  noComponent = false;
  permIndex = 0;
  driverIndex = 0;
  if (sensorDriverCount() == 0) {  // nothing to try: never precheck or power
    noComponent = true;
    goIdle(now, PROBE_RESCAN_INTERVAL_MS);
    return;
  }
  beginPrecheck(now);
}

// Internal pulls are ~45 kOhm, so these readings are only a coarse hint: a
// pulled-down node reading HIGH (coupling) or any node changing between
// samples vetoes powering. LOW readings do not prove there is no short.
void tickPrecheck(uint32_t now) {
  const uint32_t wait = precheckSample == 0 ? PRECHECK_SETTLE_MS : PRECHECK_SAMPLE_INTERVAL_MS;
  if (!elapsed(now, stateSinceMs, wait)) return;
  stateSinceMs = now;

  bool suspicious = false;
  bool inconsistent = false;
  for (uint8_t n = 0; n < PROBE_NODE_COUNT; n++) {
    const uint8_t level = digitalRead(gpio(n)) == HIGH ? HIGH : LOW;
    if (n != precheckSource && level == HIGH) suspicious = true;
    if (precheckSample == 0) {
      precheckFirst[n] = level;
    } else if (level != precheckFirst[n]) {
      inconsistent = true;
    }
  }

  if (suspicious || inconsistent) {
    status.precheck = suspicious ? PrecheckResult::SuspiciousCoupling
                                 : PrecheckResult::InconsistentReadings;
    enterFault(now);
    return;
  }

  if (++precheckSample < PRECHECK_SAMPLES) return;
  if (++precheckSource < PROBE_NODE_COUNT) {
    precheckSample = 0;
    applyPrecheckPulls(precheckSource);
    return;
  }
  resetTestPins();  // all nodes plain INPUT before powering
  status.precheck = PrecheckResult::NoObviousCoupling;
  enter(State::PowerUp, now);
}

// ---- Results ---------------------------------------------------------------

// Promotes the candidate to the active driver after enough consecutive,
// genuinely new acquisitions (probe() and readSample() completions).
void countConfirmation() {
  if (!status.detected && ++confirmations >= PROBE_CONFIRM_FRAMES) status.detected = true;
}

void onProbeSucceeded(SensorDriver *driver, uint32_t now) {
  status.consecutiveFailures = 0;
  status.lastError = nullptr;
  if (savedDriver == nullptr) {  // first success: keep candidate, stop scanning
    savedDriver = driver;
    status.hasMapping = true;
    status.mapping = PROBE_PERMUTATIONS[permIndex];
    status.componentName = driver->getName();
    confirmations = 0;
  }
  countConfirmation();
  // The mapping stays powered; telemetry arrives with the next readSample().
  enter(status.detected ? State::Monitoring : State::Confirming, now);
}

void onSampleSucceeded(const char *line1, const char *line2, uint32_t now) {
  snprintf(status.telemetry1, sizeof status.telemetry1, "%s", line1);
  snprintf(status.telemetry2, sizeof status.telemetry2, "%s", line2);
  status.readingValid = true;
  status.readingStale = false;
  status.lastReadingMs = now;
  status.consecutiveFailures = 0;
  status.lastError = nullptr;
  countConfirmation();
  enter(status.detected ? State::Monitoring : State::Confirming, now);
}

void onAttemptFailed(uint32_t now, const char *reason) {
  powerDown(activeMapping());  // pins safe first, immediately...
  currentDriver()->reset();    // ...then cancel the driver's state
  status.lastError = reason;

  if (savedDriver == nullptr) {  // discovery: try the next driver, then permutation
    if (++driverIndex < sensorDriverCount()) {
      enter(State::PowerOffWait, now);
      return;
    }
    driverIndex = 0;
    if (++permIndex < PROBE_PERMUTATION_COUNT) {
      enter(State::PowerOffWait, now);
    } else {
      noComponent = true;  // "No Component / Unknown"
      goIdle(now, PROBE_RESCAN_INTERVAL_MS);
    }
    return;
  }

  status.readingStale = true;
  confirmations = 0;  // confirmation needs consecutive acquisitions
  if (++status.consecutiveFailures < PROBE_MAX_CONSECUTIVE_FAILURES) {
    enter(State::RetryWait, now);  // retry only the saved driver and mapping
    return;
  }
  // No removal sensor exists: removal is inferred from failed acquisitions.
  clearResult();
  goIdle(now, PROBE_RESCAN_INTERVAL_MS);
}

// Pending is not a failure, unless it outlasts the driver's own timeout.
void onNotSucceeded(SensorDriver *driver, uint32_t now) {
  if (driver->getOperationStatus() != DriverOperationStatus::Pending) {
    const char *reason = driver->getFailureReason();
    onAttemptFailed(now, reason != nullptr && reason[0] != '\0' ? reason : "driver-failed");
  } else if (elapsed(now, lastAcquisitionMs, driver->getRequirements().operationTimeoutMs)) {
    onAttemptFailed(now, "operation-timeout");
  }
}

void tickProbe(uint32_t now) {
  SensorDriver *driver = currentDriver();
  const ProbeMapping &m = activeMapping();
  if (driver->probe(gpio(m.vcc), gpio(m.data), gpio(m.gnd))) {
    onProbeSucceeded(driver, now);
  } else {
    onNotSucceeded(driver, now);
  }
}

void tickSample(uint32_t now) {
  char line1[PROBE_TELEMETRY_TEXT_LEN];
  char line2[PROBE_TELEMETRY_TEXT_LEN];
  if (savedDriver->readSample(line1, line2, sizeof line1)) {
    onSampleSucceeded(line1, line2, now);
  } else {
    onNotSucceeded(savedDriver, now);
  }
}

void step(uint32_t now) {
  switch (state) {
    case State::Disabled:
    case State::Fault:
      break;  // probes stay INPUT; Fault waits for probeRequestRetry()

    case State::Idle:
      if (elapsed(now, stateSinceMs, idleDelayMs)) startDiscovery(now);
      break;

    case State::Precheck:
      tickPrecheck(now);
      break;

    case State::PowerUp:
      powerUp(activeMapping());
      enter(State::Stabilizing, now);
      break;

    case State::Stabilizing: {
      // Retries of a saved mapping also respect the acquisition spacing.
      const DriverRequirements &req = currentDriver()->getRequirements();
      if (elapsed(now, stateSinceMs, req.powerStabilizeMs) &&
          (savedDriver == nullptr || elapsed(now, lastAcquisitionMs, req.minAcquisitionIntervalMs))) {
        lastAcquisitionMs = now;
        enter(State::Probing, now);
        tickProbe(now);  // first call starts the operation this tick
      }
      break;
    }

    case State::Probing:
      tickProbe(now);
      break;

    case State::Confirming:
    case State::Monitoring: {
      const DriverRequirements &req = savedDriver->getRequirements();
      const uint32_t interval = req.sampleIntervalMs > req.minAcquisitionIntervalMs
                                    ? req.sampleIntervalMs
                                    : req.minAcquisitionIntervalMs;
      if (elapsed(now, lastAcquisitionMs, interval)) {
        lastAcquisitionMs = now;
        enter(State::Sampling, now);
        tickSample(now);
      }
      break;
    }

    case State::Sampling:
      tickSample(now);
      break;

    case State::PowerOffWait:
      // A timed wait is not proof a capacitor has discharged; nodes are never
      // shorted to discharge them.
      if (elapsed(now, stateSinceMs, PROBE_POWER_OFF_MS)) beginPrecheck(now);
      break;

    case State::RetryWait:
      if (elapsed(now, stateSinceMs, PROBE_RETRY_COOLDOWN_MS)) beginPrecheck(now);
      break;

    default:
      enterFault(now);  // unknown state: same safe path as a fault
      break;
  }
}

ProbePhase derivePhase() {
  switch (state) {
    case State::Disabled: return ProbePhase::Disabled;
    case State::Fault: return ProbePhase::Fault;
    case State::Idle: return noComponent ? ProbePhase::NoComponent : ProbePhase::Idle;
    default: break;
  }
  if (status.detected) return status.readingStale ? ProbePhase::Retrying : ProbePhase::Detected;
  return status.hasMapping ? ProbePhase::Confirming : ProbePhase::Scanning;
}

bool sameText(const char *a, const char *b) {
  return a == b || (a != nullptr && b != nullptr && strcmp(a, b) == 0);
}

bool sameForDisplay(const ProbeStatus &a, const ProbeStatus &b) {
  return a.phase == b.phase && a.scanIndex == b.scanIndex && a.hasMapping == b.hasMapping &&
         a.mapping.vcc == b.mapping.vcc && a.mapping.data == b.mapping.data &&
         a.mapping.gnd == b.mapping.gnd && sameText(a.componentName, b.componentName) &&
         a.detected == b.detected && a.readingValid == b.readingValid &&
         a.readingStale == b.readingStale && sameText(a.telemetry1, b.telemetry1) &&
         sameText(a.telemetry2, b.telemetry2) &&
         a.consecutiveFailures == b.consecutiveFailures && sameText(a.lastError, b.lastError) &&
         a.precheck == b.precheck;
}

void publish(const ProbeStatus &before) {
  status.phase = derivePhase();
  status.scanIndex =
      status.phase == ProbePhase::Scanning ? static_cast<uint8_t>(permIndex + 1) : 0;
  if (!sameForDisplay(before, status)) status.revision++;
}

const char *roleName(const ProbeMapping &m, uint8_t node) {
  if (node == m.vcc) return "VCC";
  if (node == m.data) return "DAT";
  if (node == m.gnd) return "GND";
  return "---";
}

}  // namespace

void resetTestPins() {
  // Plain INPUT: output driver off, internal pull-up and pull-down off.
  pinMode(PROBE_PIN_A, INPUT);
  pinMode(PROBE_PIN_B, INPUT);
  pinMode(PROBE_PIN_C, INPUT);
}

void initProbeEngine() {
  resetTestPins();
  for (size_t i = 0; i < sensorDriverCount(); i++) sensorDriverAt(i)->reset();

  const ProbeStatus before = status;
  const uint32_t now = millis();
  status = ProbeStatus{};
  status.revision = before.revision + 1;  // force a first redraw
  savedDriver = nullptr;
  driverIndex = 0;
  permIndex = 0;
  confirmations = 0;
  noComponent = false;

  if (PINDEX_POWERED_PROBING_ENABLED) {
    goIdle(now, 0);  // discovery starts on the first tick
  } else {
    enter(State::Disabled, now);
  }
  publish(before);
}

void probeTick() {
  const ProbeStatus before = status;
  step(millis());
  publish(before);
}

const ProbeStatus &probeStatus() { return status; }

bool probeRequestRetry() {
  if (!PINDEX_POWERED_PROBING_ENABLED || state != State::Fault) return false;
  const ProbeStatus before = status;
  startDiscovery(millis());  // full precheck, then permutation 1, driver 1
  publish(before);
  return true;
}

void formatProbeMapping(const ProbeMapping &mapping, char *buf, size_t len) {
  snprintf(buf, len, "%c:%s | %c:%s | %c:%s", kNodeLabel[0], roleName(mapping, 0),
           kNodeLabel[1], roleName(mapping, 1), kNodeLabel[2], roleName(mapping, 2));
}

const char *probePhaseName(ProbePhase phase) {
  switch (phase) {
    case ProbePhase::Disabled: return "disabled";
    case ProbePhase::Idle: return "idle";
    case ProbePhase::Scanning: return "scanning";
    case ProbePhase::Confirming: return "confirming";
    case ProbePhase::Detected: return "detected";
    case ProbePhase::Retrying: return "retrying";
    case ProbePhase::NoComponent: return "no-component/unknown";
    case ProbePhase::Fault: return "fault";
  }
  return "unknown";
}

const char *precheckResultName(PrecheckResult result) {
  switch (result) {
    case PrecheckResult::NotRun: return "not-run";
    case PrecheckResult::NoObviousCoupling: return "no-obvious-coupling";
    case PrecheckResult::SuspiciousCoupling: return "suspicious-coupling";
    case PrecheckResult::InconsistentReadings: return "inconsistent-readings";
  }
  return "unknown";
}
