#include "probe_engine.h"

#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "four_wire_backend.h"
#include "sensor_driver.h"
#include "src/drivers/driver_registry.h"

// Safety scope: plain INPUT (high impedance) only means the firmware is not
// driving the probes. It is NOT protection against shorts, overvoltage, ESD
// or charged capacitors. Three-wire powered probing drives probe GPIOs as a
// nominal 3.3V logic-level supply, not a regulated rail, and must only be
// enabled behind a reviewed protected interface (see README). Four-wire
// probing never uses a GPIO as a supply: it runs only on a backend that
// reports the required capabilities (four_wire_backend.h).
//
// Routing: the weak-pull precheck only observes. A coupled pair (contact or
// finite resistance; the digital test cannot tell which) never authorises
// powered drive across it; it is measured by weak-pull (passive) drivers
// instead. Seeing no coupling never authorises anything either.
// Inconsistent readings are retried, then treated as "unknown".
//
// The engine owns pins, power, prechecks, scheduling, confirmation and
// removal. Drivers (sensor_driver.h) are chosen by capabilities only; this
// file knows no concrete driver.

static_assert(PROBE_RETRY_COOLDOWN_MS >= PROBE_POWER_OFF_MS,
              "a retry must include the power-off interval");
static_assert(PROBE_RESCAN_INTERVAL_MS >= PROBE_POWER_OFF_MS,
              "a rescan must include the power-off interval");
static_assert(PROBE_CONFIRM_FRAMES >= 1, "confirmation needs at least one acquisition");
static_assert(PROBE_NODE_COUNT == 4, "the node and mapping tables below are written for A/B/C/D");
static_assert(PROBE_PIN_A != PROBE_PIN_B && PROBE_PIN_A != PROBE_PIN_C && PROBE_PIN_A != PROBE_PIN_D &&
                  PROBE_PIN_B != PROBE_PIN_C && PROBE_PIN_B != PROBE_PIN_D && PROBE_PIN_C != PROBE_PIN_D,
              "every socket node needs its own GPIO");
static_assert(PROBE_PERMUTATION_COUNT == 4 * 3 * 2, "three-wire: ordered triples of 4 nodes");
static_assert(PROBE_FOUR_WIRE_COUNT == 4 * 3 * 2 * 1, "four-wire: orderings of 4 nodes");
static_assert(PROBE_PAIR_COUNT == 4 * 3 / 2, "two-terminal: unordered pairs of 4 nodes");
static_assert(PROBE_PAIR_COUNT <= 8, "ProbeStatus::coupledPairs is an 8-bit pair mask");

const uint8_t PROBE_NODE_GPIO[] = {PROBE_PIN_A, PROBE_PIN_B, PROBE_PIN_C, PROBE_PIN_D};
const char PROBE_NODE_LABEL[] = {'A', 'B', 'C', 'D'};
static_assert(sizeof PROBE_NODE_GPIO == PROBE_NODE_COUNT && sizeof PROBE_NODE_LABEL == PROBE_NODE_COUNT,
              "one GPIO and one label per node");

namespace {

constexpr uint8_t A = 0, B = 1, C = 2, D = 3;  // node indices

constexpr ProbeMapping threeWire(uint8_t vcc, uint8_t data, uint8_t gnd) {
  ProbeMapping m{ProbeTopology::ThreeWirePowered, {}};  // every node Unused
  m.role[vcc] = NodeRole::Vcc;
  m.role[data] = NodeRole::Data;
  m.role[gnd] = NodeRole::Gnd;
  return m;
}

constexpr ProbeMapping fourWire(uint8_t vcc, uint8_t sig1, uint8_t sig2, uint8_t gnd) {
  ProbeMapping m{ProbeTopology::FourWirePowered, {}};
  m.role[vcc] = NodeRole::Vcc;
  m.role[sig1] = NodeRole::Sig1;
  m.role[sig2] = NodeRole::Sig2;
  m.role[gnd] = NodeRole::Gnd;
  return m;
}

constexpr ProbeMapping terminals(uint8_t x, uint8_t y) {
  ProbeMapping m{ProbeTopology::TwoTerminal, {}};
  m.role[x] = NodeRole::Terminal;
  m.role[y] = NodeRole::Terminal;
  return m;
}

}  // namespace

// The six VCC/DATA/GND patterns of a node set x < y < z, in the original
// Phase 2 scan order.
#define PATTERNS(x, y, z)                                                                    \
  threeWire(x, y, z), threeWire(z, y, x), threeWire(y, x, z), threeWire(y, z, x), \
      threeWire(x, z, y), threeWire(z, x, y)

const ProbeMapping PROBE_PERMUTATIONS[PROBE_PERMUTATION_COUNT] = {
    PATTERNS(A, B, C),  // 1-6: the original A/B/C scan, unchanged
    PATTERNS(A, B, D),  // 7-12
    PATTERNS(A, C, D),  // 13-18
    PATTERNS(B, C, D),  // 19-24
};

#undef PATTERNS

const ProbeMapping PROBE_FOUR_WIRE_MAPPINGS[PROBE_FOUR_WIRE_COUNT] = {
    // VCC, SIG1, SIG2, GND nodes.
    fourWire(A, B, C, D), fourWire(A, B, D, C), fourWire(A, C, B, D), fourWire(A, C, D, B),
    fourWire(A, D, B, C), fourWire(A, D, C, B), fourWire(B, A, C, D), fourWire(B, A, D, C),
    fourWire(B, C, A, D), fourWire(B, C, D, A), fourWire(B, D, A, C), fourWire(B, D, C, A),
    fourWire(C, A, B, D), fourWire(C, A, D, B), fourWire(C, B, A, D), fourWire(C, B, D, A),
    fourWire(C, D, A, B), fourWire(C, D, B, A), fourWire(D, A, B, C), fourWire(D, A, C, B),
    fourWire(D, B, A, C), fourWire(D, B, C, A), fourWire(D, C, A, B), fourWire(D, C, B, A),
};

const ProbeMapping PROBE_PAIRS[PROBE_PAIR_COUNT] = {
    terminals(A, B), terminals(B, C), terminals(A, C),  // the original three
    terminals(A, D), terminals(B, D), terminals(C, D),
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

const uint8_t kNoPin = 0xFF;  // never a valid GPIO; Arduino rejects it

// Every cycle tries the active protocols first (registry priority: DHT11 is
// three-wire, the ultrasonic four-wire); two-terminal (passive)
// interpretation is the fallback once no active protocol answered.
const ProbeTopology kTopologyOrder[] = {ProbeTopology::ThreeWirePowered,
                                        ProbeTopology::FourWirePowered,
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

uint8_t gpio(uint8_t node) { return PROBE_NODE_GPIO[node]; }

uint8_t gpioWithRole(const ProbeMapping &m, NodeRole role, int nth = 0) {
  const int n = probeNodeWithRole(m, role, nth);
  return n >= 0 ? PROBE_NODE_GPIO[n] : kNoPin;
}

uint8_t nodeWithRole(const ProbeMapping &m, NodeRole role) {
  const int n = probeNodeWithRole(m, role);
  return n >= 0 ? static_cast<uint8_t>(n) : kNoPin;
}

// ---- Capabilities and the attempt list -------------------------------------

// The only capability combinations the engine knows how to prepare.
bool supported(const DriverRequirements &r) {
  switch (r.topology) {
    case ProbeTopology::ThreeWirePowered:
    case ProbeTopology::FourWirePowered:
      return r.excitation == ExcitationPolicy::ProtectedActive;
    case ProbeTopology::TwoTerminal:
      return r.excitation == ExcitationPolicy::WeakPullOnly;
  }
  return false;
}

bool topologyEnabled(ProbeTopology t) {
  return t == ProbeTopology::TwoTerminal ? PINDEX_PASSIVE_PROBING_ENABLED != 0
                                         : PINDEX_POWERED_PROBING_ENABLED != 0;
}

// FourWirePowered needs a backend that reports the baseline plus the
// driver's own needs. Nothing else (no config flag) can grant them.
bool backendProvides(const DriverRequirements &r) {
  if (r.topology != ProbeTopology::FourWirePowered) return true;
  const uint16_t needed = kBackendFourWireBaseline | r.backendCapabilities;
  return (fourWireBackendCapabilities() & needed) == needed;
}

bool runnable(const DriverRequirements &r) {
  return supported(r) && topologyEnabled(r.topology) && backendProvides(r);
}

// First enabled, supported driver that runnable() excludes for lack of
// backend capabilities (reported as "hardware required"), or nullptr.
const char *hardwareRequiredDriver() {
  for (size_t i = 0; i < sensorDriverCount(); i++) {
    const SensorDriver *d = sensorDriverAt(i);
    const DriverRequirements &r = d->getRequirements();
    if (supported(r) && topologyEnabled(r.topology) && !backendProvides(r)) return d->getName();
  }
  return nullptr;
}

size_t slotCount(ProbeTopology t) {
  switch (t) {
    case ProbeTopology::ThreeWirePowered: return PROBE_PERMUTATION_COUNT;
    case ProbeTopology::FourWirePowered: return PROBE_FOUR_WIRE_COUNT;
    case ProbeTopology::TwoTerminal: break;
  }
  return PROBE_PAIR_COUNT;
}

const ProbeMapping &slotMapping(ProbeTopology t, size_t slot) {
  switch (t) {
    case ProbeTopology::ThreeWirePowered: return PROBE_PERMUTATIONS[slot];
    case ProbeTopology::FourWirePowered: return PROBE_FOUR_WIRE_MAPPINGS[slot];
    case ProbeTopology::TwoTerminal: break;
  }
  return PROBE_PAIRS[slot];
}

size_t driverCount(ProbeTopology t) {
  size_t count = 0;
  for (size_t i = 0; i < sensorDriverCount(); i++) {
    const DriverRequirements &r = sensorDriverAt(i)->getRequirements();
    if (r.topology == t && runnable(r)) count++;
  }
  return count;
}

SensorDriver *driverOfTopology(ProbeTopology t, size_t nth) {
  for (size_t i = 0; i < sensorDriverCount(); i++) {
    SensorDriver *d = sensorDriverAt(i);
    const DriverRequirements &r = d->getRequirements();
    if (r.topology == t && runnable(r) && nth-- == 0) return d;
  }
  return nullptr;
}

size_t attemptTotal() {
  size_t total = 0;
  for (ProbeTopology t : kTopologyOrder) total += driverCount(t) * slotCount(t);
  return total;
}

// Within a topology: slot-major (every driver on a mapping or pair, in
// registry order, before the next one). Pairs are enumerated once per
// cycle; a pair's two orientations are the driver's business.
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

// Powered drive is never applied across an observed coupled path. A driven
// node must show no coupling to any other node (the unused one included):
//   * VCC always (supply into a low-impedance path);
//   * four-wire SIG1/SIG2 too: SIG1 is host-driven and SIG2 sensor-driven,
//     so coupling to anything is contention. With GND the only undriven
//     role, any coupling at all vetoes a four-wire mapping.
// Three-wire DATA-GND coupling has no opposing strong drives (GND sinks;
// DATA is only pulled up or sunk), as before. No coupling seen is NOT
// permission: that comes from the protected interface, never the precheck.
bool drivenPathsUncoupled(const ProbeMapping &m, uint8_t coupled) {
  if (probeNodeWithRole(m, NodeRole::Vcc) < 0) return false;
  for (uint8_t n = 0; n < PROBE_NODE_COUNT; n++) {
    const NodeRole role = m.role[n];
    if (role != NodeRole::Vcc && role != NodeRole::Sig1 && role != NodeRole::Sig2) continue;
    for (uint8_t k = 0; k < PROBE_NODE_COUNT; k++) {
      if (k != n && (coupled & probePairBit(n, k)) != 0) return false;
    }
  }
  return true;
}

// nullptr if the attempt may be prepared, else why not.
const char *attemptBlockReason(const DriverRequirements &r, const ProbeMapping &m) {
  if (!supported(r) || r.topology != m.topology || !topologyEnabled(r.topology)) return "not-permitted";
  if (!backendProvides(r)) return "hardware-required";
  if (r.excitation == ExcitationPolicy::ProtectedActive && !drivenPathsUncoupled(m, status.coupledPairs)) {
    return "blocked-by-precheck";
  }
  return nullptr;  // WeakPullOnly: coupling is exactly what it measures
}

ProbeContext contextFor(const ProbeMapping &m) {
  ProbeContext c{};
  c.topology = m.topology;
  switch (m.topology) {
    case ProbeTopology::ThreeWirePowered:
      c.powered = {gpioWithRole(m, NodeRole::Vcc), gpioWithRole(m, NodeRole::Data),
                   gpioWithRole(m, NodeRole::Gnd)};
      break;
    case ProbeTopology::FourWirePowered:
      c.fourWire = {gpioWithRole(m, NodeRole::Vcc), gpioWithRole(m, NodeRole::Sig1),
                    gpioWithRole(m, NodeRole::Sig2), gpioWithRole(m, NodeRole::Gnd)};
      break;
    case ProbeTopology::TwoTerminal:
      c.pair.first = gpioWithRole(m, NodeRole::Terminal, 0);
      c.pair.second = gpioWithRole(m, NodeRole::Terminal, 1);
      for (int i = 0; i < PROBE_NODE_COUNT - 2; i++) c.pair.unused[i] = gpioWithRole(m, NodeRole::Unused, i);
      break;
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
// The fourth node stays plain INPUT.
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

// Three-wire: DATA first (no pull-up or drive left to back-power the part
// through its DATA pin), then VCC, then every node including GND.
// Anything else: every node to plain INPUT. resetTestPins() then switches
// the four-wire backend off, so a four-wire part loses its signals before
// its supply and ground.
void teardown(const ProbeMapping *m) {
  if (m != nullptr && m->topology == ProbeTopology::ThreeWirePowered) {
    const int data = probeNodeWithRole(*m, NodeRole::Data);
    const int vcc = probeNodeWithRole(*m, NodeRole::Vcc);
    if (data >= 0) pinMode(gpio(static_cast<uint8_t>(data)), INPUT);
    if (vcc >= 0) pinMode(gpio(static_cast<uint8_t>(vcc)), INPUT);
  }
  resetTestPins();
}

// Three-wire ProtectedActive: GPIO power-up. Four-wire: every GPIO plain
// INPUT and the backend off (break), then the backend routes this mapping
// (make); false if it refuses. WeakPullOnly: every node plain INPUT; the
// driver excites its pair itself (pull-up + open-drain LOW sink only).
bool prepareExcitation(const DriverRequirements &r, const ProbeMapping &m) {
  if (r.topology == ProbeTopology::ThreeWirePowered) {
    powerUp(m);
    return true;
  }
  resetTestPins();
  if (r.topology != ProbeTopology::FourWirePowered) return true;
  return fourWireBackendConnect(nodeWithRole(m, NodeRole::Vcc), nodeWithRole(m, NodeRole::Sig1),
                                nodeWithRole(m, NodeRole::Sig2), nodeWithRole(m, NodeRole::Gnd));
}

// ---- Result bookkeeping -----------------------------------------------------

void clearResult() {
  if (savedDriver != nullptr) savedDriver->reset();  // idempotent, never energises
  savedDriver = nullptr;
  status.hasMapping = false;
  status.mapping = ProbeMapping{};
  status.signalLabel[0] = status.signalLabel[1] = nullptr;
  status.componentName = nullptr;
  status.detected = false;
  status.readingValid = false;
  status.readingStale = false;
  status.telemetry1[0] = '\0';
  status.telemetry2[0] = '\0';
  status.consecutiveFailures = 0;
  confirmations = 0;
}

// Keeps `driver` on `m` as the candidate; confirmation starts from zero.
void saveCandidate(SensorDriver *driver, const ProbeMapping &m) {
  const DriverRequirements &r = driver->getRequirements();
  savedDriver = driver;
  status.mapping = m;
  status.hasMapping = true;
  status.signalLabel[0] = r.signalLabel[0];
  status.signalLabel[1] = r.signalLabel[1];
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
  status.hardwareRequired = hardwareRequiredDriver();  // skipped, never a scan fault
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
    saveCandidate(hit.driver, *hit.mapping);
    status.componentName = hit.driver->getName();
    status.consecutiveFailures = 0;
    enter(State::PowerOffWait, now);
    return;
  }
  noComponent = true;
  if (twoTerminalHits > 1) status.lastError = "ambiguous-pairs";  // multi-node coupling
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

// The attempt is not permitted (coupled driven path, missing backend
// capability, disabled topology); nothing was energised.
void onAttemptBlocked(uint32_t now, const char *reason) {
  if (savedDriver != nullptr) {
    onAttemptFailed(now, reason);  // never powered across it
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
    // published, so a conducting pair is never reported while another
    // node is also coupled.
    twoTerminalHits++;
    twoTerminalHitIndex = attemptIndex;
    teardown(currentMapping());
    driver->reset();
    advanceAttempt(now, true);
    return;
  }
  if (savedDriver == nullptr) saveCandidate(driver, *currentMapping());  // stop scanning
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
  // No removal sensor exists: loss of response (removal, or e.g. no echo
  // target) is inferred from failed acquisitions, never proven.
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
      if (const char *blocked = attemptBlockReason(driver->getRequirements(), *m)) {
        onAttemptBlocked(now, blocked);
        break;
      }
      if (!prepareExcitation(driver->getRequirements(), *m)) {
        onAttemptFailed(now, "backend-refused");  // torn down; next attempt or retry
        break;
      }
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
         sameText(a.signalLabel[0], b.signalLabel[0]) && sameText(a.signalLabel[1], b.signalLabel[1]) &&
         sameText(a.componentName, b.componentName) && a.detected == b.detected &&
         a.readingValid == b.readingValid && a.readingStale == b.readingStale &&
         sameText(a.telemetry1, b.telemetry1) && sameText(a.telemetry2, b.telemetry2) &&
         a.consecutiveFailures == b.consecutiveFailures && sameText(a.lastError, b.lastError) &&
         a.precheck == b.precheck && a.coupledPairs == b.coupledPairs &&
         a.measurementUnavailable == b.measurementUnavailable &&
         sameText(a.hardwareRequired, b.hardwareRequired);
}

void publish(const ProbeStatus &before) {
  status.phase = derivePhase();
  status.scanTotal = static_cast<uint16_t>(attemptTotal());
  status.scanIndex = status.phase == ProbePhase::Scanning && attemptIndex < status.scanTotal
                         ? static_cast<uint16_t>(attemptIndex + 1)
                         : 0;
  if (!sameForDisplay(before, status)) status.revision++;
}

const char *roleName(NodeRole role, const char *const *signalLabel) {
  switch (role) {
    case NodeRole::Vcc: return "VCC";
    case NodeRole::Data: return "DAT";
    case NodeRole::Gnd: return "GND";
    case NodeRole::Terminal: return "T";
    case NodeRole::Sig1: return signalLabel != nullptr && signalLabel[0] != nullptr ? signalLabel[0] : "S1";
    case NodeRole::Sig2: return signalLabel != nullptr && signalLabel[1] != nullptr ? signalLabel[1] : "S2";
    case NodeRole::Unused: break;
  }
  return "---";
}

// Appends "X:role" to buf, space-separated; truncates, always terminated.
void appendNode(char *buf, size_t len, int node, const char *role) {
  const size_t used = strlen(buf);
  if (used + 1 < len) {
    snprintf(buf + used, len - used, "%s%c:%s", used > 0 ? " " : "", PROBE_NODE_LABEL[node], role);
  }
}

}  // namespace

void resetTestPins() {
  // Plain INPUT: output driver off, internal pull-up and pull-down off.
  for (uint8_t pin : PROBE_NODE_GPIO) pinMode(pin, INPUT);
  // Then the external four-wire supply and routes, after the signals, so a
  // part is never back-fed from a GPIO once unpowered.
  fourWireBackendDisconnect();
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
  status.hardwareRequired = hardwareRequiredDriver();

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

void formatProbeMapping(const ProbeMapping &mapping, const char *const *signalLabel, char *first,
                        char *second, size_t len) {
  if (len == 0) return;
  first[0] = second[0] = '\0';
  const int t0 = probeNodeWithRole(mapping, NodeRole::Terminal, 0);
  const int t1 = probeNodeWithRole(mapping, NodeRole::Terminal, 1);
  if (mapping.topology == ProbeTopology::TwoTerminal && t0 >= 0 && t1 >= 0) {
    snprintf(first, len, "Pins: %c & %c", PROBE_NODE_LABEL[t0], PROBE_NODE_LABEL[t1]);
    for (int n = 0; n < PROBE_NODE_COUNT; n++) {
      if (mapping.role[n] == NodeRole::Unused) appendNode(second, len, n, "---");
    }
    return;
  }
  for (int n = 0; n < PROBE_NODE_COUNT; n++) {  // A, B on the first half; C, D on the second
    appendNode(n < PROBE_NODE_COUNT / 2 ? first : second, len, n, roleName(mapping.role[n], signalLabel));
  }
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
