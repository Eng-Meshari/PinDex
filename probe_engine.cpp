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
// Routing: the weak-pull precheck only observes. A coupled pair (contact or
// finite resistance; the digital test cannot tell which) never authorises
// powered drive across it; it is measured by weak-pull (passive) drivers
// instead. Inconsistent readings are retried, then treated as "unknown".
//
// The engine owns pins, power, prechecks, scheduling, confirmation and
// removal. Drivers (sensor_driver.h) are chosen by capabilities only; this
// file knows no concrete driver.

static_assert(PROBE_RETRY_COOLDOWN_MS >= PROBE_POWER_OFF_MS,
              "a retry must include the power-off interval");
static_assert(PROBE_RESCAN_INTERVAL_MS >= PROBE_POWER_OFF_MS,
              "a rescan must include the power-off interval");
static_assert(PROBE_CONFIRM_FRAMES >= 1, "confirmation needs at least one acquisition");

const ProbeMapping PROBE_PERMUTATIONS[PROBE_PERMUTATION_COUNT] = {
    // Roles of A, B, C.
    {ProbeTopology::ThreeWirePowered, {NodeRole::Vcc, NodeRole::Data, NodeRole::Gnd}},  // 1: A B C
    {ProbeTopology::ThreeWirePowered, {NodeRole::Gnd, NodeRole::Data, NodeRole::Vcc}},  // 2: C B A
    {ProbeTopology::ThreeWirePowered, {NodeRole::Data, NodeRole::Vcc, NodeRole::Gnd}},  // 3: B A C
    {ProbeTopology::ThreeWirePowered, {NodeRole::Gnd, NodeRole::Vcc, NodeRole::Data}},  // 4: B C A
    {ProbeTopology::ThreeWirePowered, {NodeRole::Vcc, NodeRole::Gnd, NodeRole::Data}},  // 5: A C B
    {ProbeTopology::ThreeWirePowered, {NodeRole::Data, NodeRole::Gnd, NodeRole::Vcc}},  // 6: C A B
};

const ProbeMapping PROBE_PAIRS[PROBE_PAIR_COUNT] = {
    {ProbeTopology::TwoTerminal, {NodeRole::Terminal, NodeRole::Terminal, NodeRole::Unused}},  // A-B
    {ProbeTopology::TwoTerminal, {NodeRole::Unused, NodeRole::Terminal, NodeRole::Terminal}},  // B-C
    {ProbeTopology::TwoTerminal, {NodeRole::Terminal, NodeRole::Unused, NodeRole::Terminal}},  // A-C
};

uint8_t probePairBit(uint8_t nodeX, uint8_t nodeY) {
  if (nodeX == nodeY || nodeX >= PROBE_NODE_COUNT || nodeY >= PROBE_NODE_COUNT) return 0;
  for (uint8_t i = 0; i < PROBE_PAIR_COUNT; i++) {
    if (PROBE_PAIRS[i].role[nodeX] == NodeRole::Terminal &&
        PROBE_PAIRS[i].role[nodeY] == NodeRole::Terminal) {
      return static_cast<uint8_t>(1u << i);
    }
  }
  return 0;
}

int probeNodeWithRole(const ProbeMapping &mapping, NodeRole role, int nth) {
  for (int n = 0; n < PROBE_NODE_COUNT; n++) {
    if (mapping.role[n] == role && nth-- == 0) return n;
  }
  return -1;
}

namespace {

// Socket labels and GPIOs are kept apart: roles are resolved per node.
const uint8_t kNodeGpio[PROBE_NODE_COUNT] = {PROBE_PIN_A, PROBE_PIN_B, PROBE_PIN_C};
const char kNodeLabel[PROBE_NODE_COUNT] = {'A', 'B', 'C'};
const uint8_t kNoPin = 0xFF;  // never a valid GPIO; Arduino rejects it

// Every cycle tries powered protocols first; two-terminal (passive)
// interpretation is the fallback once no active protocol answered.
const ProbeTopology kTopologyOrder[] = {ProbeTopology::ThreeWirePowered,
                                        ProbeTopology::TwoTerminal};

enum class State : uint8_t {
  Disabled,      // all probing compiled out
  Idle,          // probes INPUT, waiting idleDelayMs before discovery
  Precheck,      // weak-pull diagnostic, one pull-up source node at a time
  Prepare,       // check the attempt is permitted, apply its excitation
  Stabilizing,   // driver's powerStabilizeMs before the first probe() call
  Probing,       // driver probe() operation in progress
  Confirming,    // candidate kept, waiting for the next acquisition slot
  Monitoring,    // active driver kept, waiting for the next acquisition slot
  Sampling,      // driver readSample() operation in progress
  PowerOffWait,  // torn down, waiting before the next attempt
  RetryWait,     // saved driver failed: torn down, cooling down
  Fault,         // latched internal fault; probes INPUT
};

struct Attempt {
  SensorDriver *driver;
  const ProbeMapping *mapping;
};

State state = State::Disabled;
uint32_t stateSinceMs = 0;       // entry time of the state (or precheck step)
uint32_t idleDelayMs = 0;
uint32_t lastAcquisitionMs = 0;  // start of the latest probe()/readSample() operation
size_t attemptIndex = 0;         // position in this cycle's attempt list
SensorDriver *savedDriver = nullptr;  // candidate until detected, then active
uint8_t confirmations = 0;       // consecutive successful acquisitions
bool noComponent = false;        // last discovery found nothing
uint8_t precheckSource = 0;
uint8_t precheckSample = 0;
uint8_t precheckRetries = 0;
uint8_t precheckCoupled = 0;
uint8_t precheckFirst[PROBE_NODE_COUNT];
uint8_t twoTerminalHits = 0;     // pairs that behaved as a two-terminal part
size_t twoTerminalHitIndex = 0;
bool cycleMeasurementError = false;
ProbeStatus status;

bool elapsed(uint32_t now, uint32_t since, uint32_t interval) {
  return now - since >= interval;  // unsigned: correct across millis() rollover
}

void enter(State next, uint32_t now) {
  state = next;
  stateSinceMs = now;
}

uint8_t gpio(uint8_t node) { return kNodeGpio[node]; }

uint8_t gpioWithRole(const ProbeMapping &m, NodeRole role, int nth = 0) {
  const int n = probeNodeWithRole(m, role, nth);
  return n >= 0 ? kNodeGpio[n] : kNoPin;
}

// ---- Capabilities and the attempt list -------------------------------------

// The only capability combinations the engine knows how to prepare.
bool supported(const DriverRequirements &r) {
  return (r.topology == ProbeTopology::ThreeWirePowered &&
          r.excitation == ExcitationPolicy::ProtectedActive) ||
         (r.topology == ProbeTopology::TwoTerminal &&
          r.excitation == ExcitationPolicy::WeakPullOnly);
}

bool topologyEnabled(ProbeTopology t) {
  return t == ProbeTopology::ThreeWirePowered ? PINDEX_POWERED_PROBING_ENABLED != 0
                                              : PINDEX_PASSIVE_PROBING_ENABLED != 0;
}

size_t slotCount(ProbeTopology t) {
  return t == ProbeTopology::ThreeWirePowered ? PROBE_PERMUTATION_COUNT : PROBE_PAIR_COUNT;
}

const ProbeMapping &slotMapping(ProbeTopology t, size_t slot) {
  return t == ProbeTopology::ThreeWirePowered ? PROBE_PERMUTATIONS[slot] : PROBE_PAIRS[slot];
}

size_t driverCount(ProbeTopology t) {
  if (!topologyEnabled(t)) return 0;
  size_t count = 0;
  for (size_t i = 0; i < sensorDriverCount(); i++) {
    const DriverRequirements &r = sensorDriverAt(i)->getRequirements();
    if (supported(r) && r.topology == t) count++;
  }
  return count;
}

SensorDriver *driverOfTopology(ProbeTopology t, size_t nth) {
  for (size_t i = 0; i < sensorDriverCount(); i++) {
    SensorDriver *d = sensorDriverAt(i);
    const DriverRequirements &r = d->getRequirements();
    if (supported(r) && r.topology == t && nth-- == 0) return d;
  }
  return nullptr;
}

size_t attemptTotal() {
  size_t total = 0;
  for (ProbeTopology t : kTopologyOrder) total += driverCount(t) * slotCount(t);
  return total;
}

// Within a topology: slot-major (every driver on a permutation or pair, in
// registry order, before the next one). Pairs are enumerated once per cycle.
bool attemptAt(size_t index, Attempt &out) {
  for (ProbeTopology t : kTopologyOrder) {
    const size_t drivers = driverCount(t);
    const size_t span = drivers * slotCount(t);
    if (index < span) {
      out.mapping = &slotMapping(t, index / drivers);
      out.driver = driverOfTopology(t, index % drivers);
      return out.driver != nullptr;
    }
    index -= span;
  }
  return false;
}

const ProbeMapping *currentMapping() {
  if (status.hasMapping) return &status.mapping;
  Attempt a;
  return attemptAt(attemptIndex, a) ? a.mapping : nullptr;
}

// The driver being attempted (discovery) or the saved candidate/active one.
SensorDriver *currentDriver() {
  if (savedDriver != nullptr) return savedDriver;
  Attempt a;
  return attemptAt(attemptIndex, a) ? a.driver : nullptr;
}

// Powered drive is never applied across an observed coupled path: the VCC
// node must show no coupling to either other node. DATA-GND coupling has no
// opposing strong drives (GND sinks; DATA is only pulled up or sunk).
bool vccPathsUncoupled(const ProbeMapping &m, uint8_t coupled) {
  const int vcc = probeNodeWithRole(m, NodeRole::Vcc);
  if (vcc < 0) return false;
  for (uint8_t n = 0; n < PROBE_NODE_COUNT; n++) {
    if (n != vcc && (coupled & probePairBit(static_cast<uint8_t>(vcc), n)) != 0) return false;
  }
  return true;
}

bool attemptPermitted(const DriverRequirements &r, const ProbeMapping &m) {
  if (!supported(r) || r.topology != m.topology || !topologyEnabled(r.topology)) return false;
  if (r.excitation == ExcitationPolicy::ProtectedActive) {
    return vccPathsUncoupled(m, status.coupledPairs);
  }
  return true;  // WeakPullOnly: coupling is exactly what it measures
}

ProbeContext contextFor(const ProbeMapping &m) {
  ProbeContext c{};
  c.topology = m.topology;
  if (m.topology == ProbeTopology::ThreeWirePowered) {
    c.powered = {gpioWithRole(m, NodeRole::Vcc), gpioWithRole(m, NodeRole::Data),
                 gpioWithRole(m, NodeRole::Gnd)};
  } else {
    c.pair = {gpioWithRole(m, NodeRole::Terminal, 0), gpioWithRole(m, NodeRole::Terminal, 1),
              gpioWithRole(m, NodeRole::Unused)};
  }
  return c;
}

// ---- Pins ------------------------------------------------------------------

// ESP32 core 3.x: digitalWrite() on a pin already set up by pinMode() only
// sets the output latch via gpio_set_level() (see esp32-hal-gpio.c), and
// pinMode() -> ESP-IDF gpio_config() enables the driver without rewriting
// that latch. Writing the level first means the driver turns on at that
// level, never briefly at the opposite one. Confirm with a scope (README).
void driveNode(uint8_t node, uint8_t level) {
  digitalWrite(gpio(node), level);
  pinMode(gpio(node), OUTPUT);
}

// ThreeWirePowered only; any other mapping leaves every node plain INPUT.
void powerUp(const ProbeMapping &m) {
  resetTestPins();
  const int vcc = probeNodeWithRole(m, NodeRole::Vcc);
  const int data = probeNodeWithRole(m, NodeRole::Data);
  const int gnd = probeNodeWithRole(m, NodeRole::Gnd);
  if (m.topology != ProbeTopology::ThreeWirePowered || vcc < 0 || data < 0 || gnd < 0) return;
  driveNode(static_cast<uint8_t>(gnd), LOW);  // GND first, so VCC never rises without a return
  driveNode(static_cast<uint8_t>(vcc), HIGH);
  pinMode(gpio(static_cast<uint8_t>(data)), INPUT_PULLUP);
}

// Powered mappings: DATA first (no pull-up or drive left to back-power the
// part through its DATA pin), then VCC, then every node including GND.
// Anything else: every node straight to plain INPUT.
void teardown(const ProbeMapping *m) {
  if (m != nullptr && m->topology == ProbeTopology::ThreeWirePowered) {
    const int data = probeNodeWithRole(*m, NodeRole::Data);
    const int vcc = probeNodeWithRole(*m, NodeRole::Vcc);
    if (data >= 0) pinMode(gpio(static_cast<uint8_t>(data)), INPUT);
    if (vcc >= 0) pinMode(gpio(static_cast<uint8_t>(vcc)), INPUT);
  }
  resetTestPins();
}

// The VCC-HIGH/GND-LOW path runs only for ProtectedActive drivers; a
// WeakPullOnly driver gets every node as plain INPUT and excites its pair
// itself (pull-up + open-drain LOW sink only).
void prepareExcitation(const DriverRequirements &r, const ProbeMapping &m) {
  if (r.excitation == ExcitationPolicy::ProtectedActive) {
    powerUp(m);
  } else {
    resetTestPins();
  }
}

// ---- Result bookkeeping -----------------------------------------------------

void clearResult() {
  if (savedDriver != nullptr) savedDriver->reset();  // idempotent, never energises
  savedDriver = nullptr;
  status.hasMapping = false;
  status.mapping = ProbeMapping{};
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
  clearResult();  // nothing is retried until a request
  enter(State::Fault, now);
}

// ---- Precheck ----------------------------------------------------------------

// Precheck step: resetTestPins(), then one node pulled up, the others down.
void applyPrecheckPulls(uint8_t source) {
  resetTestPins();
  for (uint8_t n = 0; n < PROBE_NODE_COUNT; n++) {
    pinMode(gpio(n), n == source ? INPUT_PULLUP : INPUT_PULLDOWN);
  }
}

void startPrecheckPass(uint32_t now) {
  precheckSource = 0;
  precheckSample = 0;
  precheckCoupled = 0;
  applyPrecheckPulls(precheckSource);
  enter(State::Precheck, now);
}

void beginPrecheck(uint32_t now) {
  precheckRetries = 0;
  startPrecheckPass(now);
}

void onAttemptFailed(uint32_t now, const char *reason);

void onPrecheckInconclusive(uint32_t now) {
  resetTestPins();
  status.precheck = PrecheckResult::InconsistentReadings;
  status.coupledPairs = 0;
  if (savedDriver != nullptr) {
    onAttemptFailed(now, "precheck-inconsistent");
    return;
  }
  // Ambiguous/noisy: no safety is inferred and nothing runs this cycle.
  noComponent = true;
  status.lastError = "precheck-inconsistent";
  goIdle(now, PROBE_RESCAN_INTERVAL_MS);
}

// Internal pulls are ~45 kOhm, so these readings are only a coarse hint: a
// pulled-down node reading HIGH marks that pair as coupled (contact or
// finite R alike). LOW readings do not prove there is no short.
void tickPrecheck(uint32_t now) {
  const uint32_t wait = precheckSample == 0 ? PRECHECK_SETTLE_MS : PRECHECK_SAMPLE_INTERVAL_MS;
  if (!elapsed(now, stateSinceMs, wait)) return;
  stateSinceMs = now;

  bool inconsistent = false;
  for (uint8_t n = 0; n < PROBE_NODE_COUNT; n++) {
    const uint8_t level = digitalRead(gpio(n)) == HIGH ? HIGH : LOW;
    if (n != precheckSource && level == HIGH) precheckCoupled |= probePairBit(precheckSource, n);
    if (precheckSample == 0) {
      precheckFirst[n] = level;
    } else if (level != precheckFirst[n]) {
      inconsistent = true;
    }
  }

  if (inconsistent) {
    if (precheckRetries++ < PRECHECK_MAX_RETRIES) {
      startPrecheckPass(now);  // bounded diagnostic retry
    } else {
      onPrecheckInconclusive(now);
    }
    return;
  }

  if (++precheckSample < PRECHECK_SAMPLES) return;
  if (++precheckSource < PROBE_NODE_COUNT) {
    precheckSample = 0;
    applyPrecheckPulls(precheckSource);
    return;
  }
  resetTestPins();  // all nodes plain INPUT before any excitation
  status.coupledPairs = precheckCoupled;
  status.precheck = precheckCoupled != 0 ? PrecheckResult::CouplingObserved
                                         : PrecheckResult::NoObviousCoupling;
  enter(State::Prepare, now);
}

// ---- Discovery flow ------------------------------------------------------------

void startDiscovery(uint32_t now) {
  clearResult();
  noComponent = false;
  attemptIndex = 0;
  twoTerminalHits = 0;
  cycleMeasurementError = false;
  status.measurementUnavailable = false;
  if (attemptTotal() == 0) {  // nothing to try: never precheck or excite
    noComponent = true;
    goIdle(now, PROBE_RESCAN_INTERVAL_MS);
    return;
  }
  beginPrecheck(now);
}

void finishCycle(uint32_t now) {
  Attempt hit;
  if (twoTerminalHits == 1 && attemptAt(twoTerminalHitIndex, hit)) {
    // Exactly one pair behaved as a two-terminal part: make it the
    // candidate and re-probe it from a fresh precheck (the scan result
    // itself is not reused as a confirming acquisition).
    savedDriver = hit.driver;
    status.mapping = *hit.mapping;
    status.hasMapping = true;
    status.componentName = hit.driver->getName();
    status.consecutiveFailures = 0;
    confirmations = 0;
    enter(State::PowerOffWait, now);
    return;
  }
  noComponent = true;
  if (twoTerminalHits > 1) status.lastError = "ambiguous-pairs";  // 3-node coupling
  status.measurementUnavailable = cycleMeasurementError && twoTerminalHits == 0;
  goIdle(now, PROBE_RESCAN_INTERVAL_MS);
}

// `energised`: whether the finished attempt applied any excitation, which
// earns a power-off interval before the next one.
void advanceAttempt(uint32_t now, bool energised) {
  if (++attemptIndex < attemptTotal()) {
    if (energised) {
      enter(State::PowerOffWait, now);
    } else {
      beginPrecheck(now);
    }
    return;
  }
  finishCycle(now);
}

// The precheck forbids this attempt (coupled VCC path or disabled topology).
void onAttemptBlocked(uint32_t now) {
  if (savedDriver != nullptr) {
    onAttemptFailed(now, "blocked-by-precheck");  // never powered across it
    return;
  }
  advanceAttempt(now, false);
}

// Promotes the candidate to the active driver after enough consecutive,
// genuinely new acquisitions (probe() and readSample() completions).
void countConfirmation() {
  if (!status.detected && ++confirmations >= PROBE_CONFIRM_FRAMES) status.detected = true;
}

void onProbeSucceeded(SensorDriver *driver, uint32_t now) {
  status.consecutiveFailures = 0;
  status.lastError = nullptr;
  if (savedDriver == nullptr && driver->getRequirements().topology == ProbeTopology::TwoTerminal) {
    // Two-terminal results are judged over every pair before one is
    // published, so a conducting pair is never reported while the third
    // node is also coupled.
    twoTerminalHits++;
    twoTerminalHitIndex = attemptIndex;
    teardown(currentMapping());
    driver->reset();
    advanceAttempt(now, true);
    return;
  }
  if (savedDriver == nullptr) {  // first success: keep candidate, stop scanning
    const ProbeMapping *m = currentMapping();
    savedDriver = driver;
    status.mapping = *m;
    status.hasMapping = true;
    confirmations = 0;
  }
  status.componentName = driver->getName();
  countConfirmation();
  // The mapping stays prepared; telemetry arrives with the next readSample().
  enter(status.detected ? State::Monitoring : State::Confirming, now);
}

void onSampleSucceeded(const char *line1, const char *line2, uint32_t now) {
  snprintf(status.telemetry1, sizeof status.telemetry1, "%s", line1);
  snprintf(status.telemetry2, sizeof status.telemetry2, "%s", line2);
  status.componentName = savedDriver->getName();  // classification may evolve
  status.readingValid = true;
  status.readingStale = false;
  status.lastReadingMs = now;
  status.consecutiveFailures = 0;
  status.lastError = nullptr;
  countConfirmation();
  enter(status.detected ? State::Monitoring : State::Confirming, now);
}

void onAttemptFailed(uint32_t now, const char *reason) {
  SensorDriver *driver = currentDriver();
  const bool measurementError = driver != nullptr && driver->failureIsMeasurementError();
  teardown(currentMapping());       // pins safe first, immediately...
  if (driver != nullptr) driver->reset();  // ...then cancel the driver's state
  status.lastError = reason;

  if (savedDriver == nullptr) {  // discovery: next attempt
    if (measurementError) cycleMeasurementError = true;
    advanceAttempt(now, true);
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
  const ProbeMapping *m = currentMapping();
  if (driver == nullptr || m == nullptr) {
    enterFault(now);
    return;
  }
  if (driver->probe(contextFor(*m))) {
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

    case State::Prepare: {
      SensorDriver *driver = currentDriver();
      const ProbeMapping *m = currentMapping();
      if (driver == nullptr || m == nullptr) {
        enterFault(now);
        break;
      }
      if (!attemptPermitted(driver->getRequirements(), *m)) {
        onAttemptBlocked(now);
        break;
      }
      prepareExcitation(driver->getRequirements(), *m);
      enter(State::Stabilizing, now);
      break;
    }

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

bool sameMapping(const ProbeMapping &a, const ProbeMapping &b) {
  return a.topology == b.topology && memcmp(a.role, b.role, sizeof a.role) == 0;
}

bool sameForDisplay(const ProbeStatus &a, const ProbeStatus &b) {
  return a.phase == b.phase && a.scanIndex == b.scanIndex && a.scanTotal == b.scanTotal &&
         a.hasMapping == b.hasMapping && sameMapping(a.mapping, b.mapping) &&
         sameText(a.componentName, b.componentName) && a.detected == b.detected &&
         a.readingValid == b.readingValid && a.readingStale == b.readingStale &&
         sameText(a.telemetry1, b.telemetry1) && sameText(a.telemetry2, b.telemetry2) &&
         a.consecutiveFailures == b.consecutiveFailures && sameText(a.lastError, b.lastError) &&
         a.precheck == b.precheck && a.coupledPairs == b.coupledPairs &&
         a.measurementUnavailable == b.measurementUnavailable;
}

void publish(const ProbeStatus &before) {
  status.phase = derivePhase();
  status.scanTotal = static_cast<uint8_t>(attemptTotal());
  status.scanIndex = status.phase == ProbePhase::Scanning && attemptIndex < status.scanTotal
                         ? static_cast<uint8_t>(attemptIndex + 1)
                         : 0;
  if (!sameForDisplay(before, status)) status.revision++;
}

const char *roleName(NodeRole role) {
  switch (role) {
    case NodeRole::Vcc: return "VCC";
    case NodeRole::Data: return "DAT";
    case NodeRole::Gnd: return "GND";
    case NodeRole::Terminal: return "T";
    case NodeRole::Unused: break;
  }
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
  attemptIndex = 0;
  confirmations = 0;
  noComponent = false;
  twoTerminalHits = 0;
  cycleMeasurementError = false;

  if (PINDEX_POWERED_PROBING_ENABLED || PINDEX_PASSIVE_PROBING_ENABLED) {
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
  if (state != State::Fault) return false;
  const ProbeStatus before = status;
  startDiscovery(millis());  // full precheck, then the first attempt
  publish(before);
  return true;
}

bool probeRequestClear() {
  if (state == State::Disabled) return false;
  const ProbeStatus before = status;
  teardown(currentMapping());                         // pins safe first
  if (SensorDriver *d = currentDriver()) d->reset();  // then cancel the driver
  startDiscovery(millis());
  publish(before);
  return true;
}

bool probeSendCommand(char command) {
  if (savedDriver == nullptr || !status.detected) return false;
  return savedDriver->handleUserCommand(command);
}

void formatProbeMapping(const ProbeMapping &mapping, char *buf, size_t len) {
  if (mapping.topology == ProbeTopology::TwoTerminal) {
    const int t0 = probeNodeWithRole(mapping, NodeRole::Terminal, 0);
    const int t1 = probeNodeWithRole(mapping, NodeRole::Terminal, 1);
    const int unused = probeNodeWithRole(mapping, NodeRole::Unused);
    if (t0 >= 0 && t1 >= 0 && unused >= 0) {
      snprintf(buf, len, "Pins: %c & %c, %c:---", kNodeLabel[t0], kNodeLabel[t1],
               kNodeLabel[unused]);
      return;
    }
  }
  snprintf(buf, len, "%c:%s | %c:%s | %c:%s", kNodeLabel[0], roleName(mapping.role[0]),
           kNodeLabel[1], roleName(mapping.role[1]), kNodeLabel[2], roleName(mapping.role[2]));
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
    case PrecheckResult::CouplingObserved: return "coupling-observed";
    case PrecheckResult::InconsistentReadings: return "inconsistent-readings";
  }
  return "unknown";
}
