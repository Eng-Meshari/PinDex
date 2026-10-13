#include "probe_engine.h"

#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "four_wire_backend.h"
#include "sensor_driver.h"
#include "src/drivers/driver_registry.h"
#include "src/drivers/weak_pull_meter.h"

// Safety scope: plain INPUT (high impedance) only means the firmware is not
// driving the probes. It is NOT protection against shorts, overvoltage, ESD
// or charged capacitors. Three-wire powered probing drives probe GPIOs as a
// nominal 3.3V logic-level supply, not a regulated rail, and must only be
// enabled behind a reviewed protected interface (see README). Four-wire
// probing runs only on a backend that reports the required capabilities
// (four_wire_backend.h), except the Supply3V3 profile's experimental GPIO
// path (PINDEX_I2C_GPIO_POWER_ENABLED, off by default), which is not
// protection either.
//
// Teardown order (releaseAttempt()): the driver releases its signals and
// peripherals while still powered, then the supply is removed, then every
// node returns to plain INPUT.
//
// Routing: the weak-pull precheck only observes. A coupled pair (contact or
// finite resistance; the digital test cannot tell which) never authorises
// powered drive across it; it is measured by weak-pull (passive) drivers
// instead. Seeing no coupling never authorises anything either.
// Inconsistent readings are retried, then that attempt alone is skipped as
// "unknown" (never powered) and discovery continues. The one
// narrow exception (decideCoupling()): an open-drain Supply3V3 four-wire
// candidate whose VCC is coupled only to its signals, where weak-pull ADC
// measurements in this observation cycle show a supported pull-up network.
//
// The engine owns pins, power, prechecks, scheduling, confirmation and
// removal. Drivers (sensor_driver.h) are chosen by capabilities only; this
// file knows no concrete driver.
//
// Evidence: a success whose driver reports EvidenceLevel::Candidate (e.g. a
// stable digital level) never stops discovery. Its attempt is torn down and
// remembered; the cycle runs on, and any Validated success (DHT frame,
// echoes, I2C ACKs) supersedes every candidate. At the end of a cycle a
// single passive pair still wins; otherwise one candidate is re-probed and
// shown as tentative, and several are reported as ambiguous and held for
// the user to pick one (probeSelectCandidate()) or clear. A pick goes
// through the same power-off wait, precheck and vetoes as any attempt.

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
static_assert(PROBE_MAX_CANDIDATES <= 255, "ProbeStatus::candidateCount is 8-bit");

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
  Classify,      // weak-pull ADC measurement of the pairs the coupling policy needs
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
// Attempt indices of this cycle's Candidate successes (status.candidateCount
// of them). ponytail: only the three-wire binary driver reports Candidate,
// so 24 slots suffice; a later one is dropped (the cycle is ambiguous anyway).
uint16_t candidateAttempt[PROBE_MAX_CANDIDATES];
bool awaitingSelection = false;  // several candidates: no rescan until a pick or a clear
bool cycleMeasurementError = false;
ProbeStatus status;

// Pair classification for the coupling policy. Scoped to one precheck
// pass (cleared when the next pass starts), so evidence never outlives a
// power, mapping or socket change: every attempt runs its own precheck.
WeakPullMeter classifyMeter;
uint8_t classifyPending = 0;  // PROBE_PAIRS bits still to measure
uint8_t classifyPair = 0;     // PROBE_PAIRS index being measured
bool classifyReversed = false;
uint16_t classifyMv[2];
uint16_t classifySpread[2];
uint8_t measuredPairs = 0;    // PROBE_PAIRS bits classified in this pass
PairEvidence evidence[PROBE_PAIR_COUNT];

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
      return r.excitation == ExcitationPolicy::ProtectedActive && r.supply == SupplyProfile::GpioLogic;
    case ProbeTopology::FourWirePowered:
      return r.excitation == ExcitationPolicy::ProtectedActive &&
             (r.supply == SupplyProfile::Backend5V || r.supply == SupplyProfile::Supply3V3);
    case ProbeTopology::TwoTerminal:
      return r.excitation == ExcitationPolicy::WeakPullOnly && r.supply == SupplyProfile::None;
  }
  return false;
}

bool topologyEnabled(ProbeTopology t) {
  return t == ProbeTopology::TwoTerminal ? PINDEX_PASSIVE_PROBING_ENABLED != 0
                                         : PINDEX_POWERED_PROBING_ENABLED != 0;
}

// How an attempt is powered.
enum class PowerPath : uint8_t { None, Gpio, Backend, Unavailable };

// kBackend* bit for a backend-supplied profile, else 0.
uint16_t backendSupplyBit(SupplyProfile s) {
  switch (s) {
    case SupplyProfile::Backend5V: return kBackendSupply5V;
    case SupplyProfile::Supply3V3: return kBackendSupply3V3;
    case SupplyProfile::None:
    case SupplyProfile::GpioLogic: break;
  }
  return 0;
}

// A protected backend is used only if it reports the baseline plus the
// profile's own supply. The experimental GPIO profile is the fallback for
// Supply3V3 alone, and only when its compile-time setting is 1; nothing
// grants it Backend5V. No runtime observation (e.g. an address hint) can
// change this.
PowerPath powerPath(const DriverRequirements &r) {
  switch (r.supply) {
    case SupplyProfile::None: return PowerPath::None;
    case SupplyProfile::GpioLogic: return PowerPath::Gpio;
    case SupplyProfile::Backend5V:
    case SupplyProfile::Supply3V3: {
      const uint16_t needed = kBackendFourWireBaseline | backendSupplyBit(r.supply);
      if ((fourWireBackendCapabilities() & needed) == needed) return PowerPath::Backend;
      if (r.supply == SupplyProfile::Supply3V3 && PINDEX_I2C_GPIO_POWER_ENABLED) return PowerPath::Gpio;
      break;
    }
  }
  return PowerPath::Unavailable;
}

bool runnable(const DriverRequirements &r) {
  return supported(r) && topologyEnabled(r.topology) && powerPath(r) != PowerPath::Unavailable;
}

// Bit i: registered driver i is enabled and supported but has no eligible
// supply (reported as "hardware required"; it is skipped, never a fault).
uint8_t hardwareRequiredMask() {
  uint8_t mask = 0;
  for (size_t i = 0; i < sensorDriverCount() && i < 8; i++) {
    const DriverRequirements &r = sensorDriverAt(i)->getRequirements();
    if (supported(r) && topologyEnabled(r.topology) && powerPath(r) == PowerPath::Unavailable) {
      mask |= static_cast<uint8_t>(1u << i);
    }
  }
  return mask;
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

// The tables use Data; a driver whose DATA is the module's output gets Out
// there instead: no engine pull-up, released at teardown, shown as OUT.
ProbeMapping rolesFor(const DriverRequirements &r, ProbeMapping m) {
  if (r.dataIsOutput && m.topology == ProbeTopology::ThreeWirePowered) {
    for (NodeRole &role : m.role) {
      if (role == NodeRole::Data) role = NodeRole::Out;
    }
  }
  return m;
}

const ProbeMapping *currentMapping() {
  if (status.hasMapping) return &status.mapping;
  static ProbeMapping attempt;  // the table entry with its driver's roles
  Attempt a;
  if (!attemptAt(attemptIndex, a)) return nullptr;
  attempt = rolesFor(a.driver->getRequirements(), *a.mapping);
  return &attempt;
}

// The driver being attempted (discovery) or the saved candidate/active one.
SensorDriver *currentDriver() {
  if (savedDriver != nullptr) return savedDriver;
  Attempt a;
  return attemptAt(attemptIndex, a) ? a.driver : nullptr;
}

// The existing rule: powered drive is never applied across an observed
// coupled path. A driven node must show no coupling to any other node
// (unused ones included):
//   * VCC always: a coupled VCC path is a known or suspected low-impedance
//     supply path (to GND or into a signal the host sinks), and the
//     digital test cannot tell a finite resistance from a short;
//   * SIG1/SIG2 too unless the driver declares them open-drain: a
//     push-pull signal (ultrasonic TRIG; ECHO is sensor-driven) coupled to
//     anything is contention, so any coupling vetoes those mappings.
// Open-drain signals (DHT DATA, I2C SDA/SCL) are only released or sunk and
// GND only sinks, so couplings among them meet no opposing strong drive.
// No coupling seen is NOT permission: that comes from the supply profile
// and the protected interface, never from the precheck.
bool drivenPathsUncoupled(const ProbeMapping &m, uint8_t coupled, bool openDrainSignals) {
  if (probeNodeWithRole(m, NodeRole::Vcc) < 0) return false;
  for (uint8_t n = 0; n < PROBE_NODE_COUNT; n++) {
    const NodeRole role = m.role[n];
    const bool pushPullSignal = !openDrainSignals && (role == NodeRole::Sig1 || role == NodeRole::Sig2);
    if (role != NodeRole::Vcc && !pushPullSignal) continue;
    for (uint8_t k = 0; k < PROBE_NODE_COUNT; k++) {
      if (k != n && (coupled & probePairBit(n, k)) != 0) return false;
    }
  }
  return true;
}

// The capability combination the pull-up exception exists for: an
// engine-switched 3.3 V four-wire supply whose two signals are only ever
// released or sunk (I2C-like). Never a driver name.
bool pullupExceptionEligible(const DriverRequirements &r) {
  return r.topology == ProbeTopology::FourWirePowered && r.excitation == ExcitationPolicy::ProtectedActive &&
         r.supply == SupplyProfile::Supply3V3 && r.openDrainSignals;
}

// Index into PROBE_PAIRS of the pair of nodes x and y, or -1.
int pairIndex(int x, int y) {
  if (x < 0 || y < 0) return -1;
  const uint8_t bit = probePairBit(static_cast<uint8_t>(x), static_cast<uint8_t>(y));
  for (int i = 0; i < PROBE_PAIR_COUNT; i++) {
    if (bit == (1u << i)) return i;
  }
  return -1;
}

// nullptr if the attempt's capabilities and supply are permitted, else why
// not. The coupling policy (decideCoupling()) comes after this.
const char *attemptBlockReason(const DriverRequirements &r, const ProbeMapping &m) {
  if (!supported(r) || r.topology != m.topology || !topologyEnabled(r.topology)) return "not-permitted";
  if (powerPath(r) == PowerPath::Unavailable) return "hardware-required";
  return nullptr;
}

ProbeContext contextFor(const ProbeMapping &m) {
  ProbeContext c{};
  c.topology = m.topology;
  switch (m.topology) {
    case ProbeTopology::ThreeWirePowered: {
      const uint8_t data = gpioWithRole(m, NodeRole::Data);
      c.powered = {gpioWithRole(m, NodeRole::Vcc), data != kNoPin ? data : gpioWithRole(m, NodeRole::Out),
                   gpioWithRole(m, NodeRole::Gnd)};
      break;
    }
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

// GPIO power (three-wire, and the experimental four-wire I2C profile):
// every node plain INPUT and the backend off first (break), then GND LOW so
// VCC never rises without a return, then VCC HIGH, each latch written
// before its output turns on. Three-wire DATA gets the pull-up; OUT,
// four-wire signals and every unused node stay plain INPUT.
void powerUpFromGpio(const ProbeMapping &m) {
  resetTestPins();
  const int vcc = probeNodeWithRole(m, NodeRole::Vcc);
  const int gnd = probeNodeWithRole(m, NodeRole::Gnd);
  if (vcc < 0 || gnd < 0) return;
  driveNode(static_cast<uint8_t>(gnd), LOW);
  driveNode(static_cast<uint8_t>(vcc), HIGH);
  const int data = probeNodeWithRole(m, NodeRole::Data);
  if (data >= 0) pinMode(gpio(static_cast<uint8_t>(data)), INPUT_PULLUP);
}

// Signals first (DATA/OUT/SIG1/SIG2: no pull-up or drive left to back-power
// the part), then VCC, then every node including GND to plain INPUT;
// resetTestPins() then switches the four-wire backend off. The driver has
// already released its own lines and peripherals (releaseAttempt()).
void teardown(const ProbeMapping *m) {
  if (m != nullptr) {
    for (uint8_t n = 0; n < PROBE_NODE_COUNT; n++) {
      const NodeRole role = m->role[n];
      if (role == NodeRole::Data || role == NodeRole::Out || role == NodeRole::Sig1 || role == NodeRole::Sig2) {
        pinMode(gpio(n), INPUT);
      }
    }
    const int vcc = probeNodeWithRole(*m, NodeRole::Vcc);
    if (vcc >= 0) pinMode(gpio(static_cast<uint8_t>(vcc)), INPUT);
  }
  resetTestPins();
}

// Prepares the attempt's supply path. Backend: every GPIO plain INPUT and
// the backend off (break), then the backend routes this mapping with the
// profile's supply (make); false if it refuses. None: every node plain
// INPUT; a weak-pull driver excites its pair itself.
bool prepareExcitation(const DriverRequirements &r, const ProbeMapping &m) {
  switch (powerPath(r)) {
    case PowerPath::Gpio:
      powerUpFromGpio(m);
      return true;
    case PowerPath::Backend:
      resetTestPins();
      return fourWireBackendConnect(backendSupplyBit(r.supply), nodeWithRole(m, NodeRole::Vcc),
                                    nodeWithRole(m, NodeRole::Sig1), nodeWithRole(m, NodeRole::Sig2),
                                    nodeWithRole(m, NodeRole::Gnd));
    case PowerPath::None:
      resetTestPins();
      return true;
    case PowerPath::Unavailable:
      break;
  }
  resetTestPins();
  return false;
}

// Driver first: it releases its lines, interrupts and peripherals (e.g.
// ends its I2C controller) while the attempt is still powered. Then supply
// off and every node plain INPUT. False if the driver could not release
// something it acquired: its ownership is uncertain.
bool releaseAttempt(SensorDriver *driver, const ProbeMapping *m) {
  if (driver != nullptr) driver->reset();
  teardown(m);
  return driver == nullptr || driver->resourcesReleased();
}

// ---- Result bookkeeping -----------------------------------------------------

void clearResult() {
  if (savedDriver != nullptr) savedDriver->reset();  // idempotent, never energises
  savedDriver = nullptr;
  status.hasMapping = false;
  status.mapping = ProbeMapping{};
  status.signalLabel[0] = status.signalLabel[1] = nullptr;
  status.componentName = nullptr;
  status.evidence = EvidenceLevel::None;
  status.detected = false;
  status.readingValid = false;
  status.readingStale = false;
  status.telemetry1[0] = '\0';
  status.telemetry2[0] = '\0';
  status.consecutiveFailures = 0;
  status.candidateSelected = 0;
  confirmations = 0;
}

void clearCandidates() {
  status.candidateCount = 0;
  status.candidateSelected = 0;
  awaitingSelection = false;
}

// Keeps `driver` on `m` as the candidate; confirmation starts from zero.
void saveCandidate(SensorDriver *driver, const ProbeMapping &m) {
  const DriverRequirements &r = driver->getRequirements();
  savedDriver = driver;
  status.mapping = rolesFor(r, m);
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
  releaseAttempt(currentDriver(), nullptr);  // cancel anything in flight, then every node INPUT
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
  measuredPairs = 0;  // a new observation cycle: earlier classifications are void
  memset(evidence, 0, sizeof evidence);
  applyPrecheckPulls(precheckSource);
  enter(State::Precheck, now);
}

void beginPrecheck(uint32_t now) {
  precheckRetries = 0;
  startPrecheckPass(now);
}

void onAttemptFailed(uint32_t now, const char *reason);
void onAttemptBlocked(uint32_t now, const char *reason);

// Still inconsistent after every retry: unknown, so this attempt is never
// powered. Pulls off first (every node plain INPUT). A saved mapping counts
// a failed acquisition as before; discovery skips only this candidate, and
// the next one starts its own fresh precheck (nothing observed here is
// reused).
void onPrecheckInconclusive(uint32_t now) {
  resetTestPins();
  status.precheck = PrecheckResult::InconsistentReadings;
  status.coupledPairs = 0;
  status.lastError = "precheck-inconsistent";
  if (savedDriver == nullptr) {
    status.inconclusiveCount++;
    status.inconclusiveAttempt = static_cast<uint16_t>(attemptIndex + 1);
  }
  onAttemptBlocked(now, "precheck-inconsistent");
}

// Internal pulls are ~45 kOhm, so these readings are only a coarse hint: a
// pulled-down node reading HIGH marks that pair as coupled (contact or
// finite R alike). LOW readings do not prove there is no short.
// At most one sample set per call, each PRECHECK_SETTLE_MS after the pull
// change or PRECHECK_SAMPLE_INTERVAL_MS after the previous set, so a
// stalled loop never takes catch-up samples.
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
    // One initial pass plus at most PRECHECK_MAX_RETRIES retries, each a
    // whole new pass from the first source node.
    if (precheckRetries < PRECHECK_MAX_RETRIES) {
      precheckRetries++;
      startPrecheckPass(now);
    } else {
      onPrecheckInconclusive(now);
    }
    return;
  }

  if (++precheckSample < PRECHECK_SAMPLES) return;
  if (++precheckSource < PROBE_NODE_COUNT) {
    precheckSample = 0;
    applyPrecheckPulls(precheckSource);  // settles from `now` (stateSinceMs above)
    return;
  }
  resetTestPins();  // all nodes plain INPUT before any excitation
  status.coupledPairs = precheckCoupled;
  status.precheck = precheckCoupled != 0 ? PrecheckResult::CouplingObserved
                                         : PrecheckResult::NoObviousCoupling;
  enter(State::Prepare, now);
}

// ---- Pair classification (weak-pull ADC, supply off) -------------------------

// One orientation of PROBE_PAIRS[classifyPair]: sense on its first
// terminal (reversed: its second), open-drain sink on the other. The
// other two nodes stay plain INPUT. False if the ADC could not be set up.
bool startClassifyOrientation(bool reversed) {
  classifyReversed = reversed;
  const ProbeMapping &p = PROBE_PAIRS[classifyPair];
  const uint8_t first = gpioWithRole(p, NodeRole::Terminal, 0);
  const uint8_t second = gpioWithRole(p, NodeRole::Terminal, 1);
  return classifyMeter.start(reversed ? second : first, reversed ? first : second, PULLUP_CLASSIFY_SETTLE_MS);
}

// Starts the next pending pair (a pair whose ADC setup fails stays a
// MeasurementError), or restores every node and returns to Prepare.
void startNextPair(uint32_t now) {
  while (classifyPending != 0) {
    classifyPair = 0;
    while ((classifyPending & (1u << classifyPair)) == 0) classifyPair++;
    classifyPending &= static_cast<uint8_t>(~(1u << classifyPair));
    measuredPairs |= static_cast<uint8_t>(1u << classifyPair);
    evidence[classifyPair] = PairEvidence{CouplingClassification::MeasurementError, {0, 0}};
    if (startClassifyOrientation(false)) return;
  }
  resetTestPins();  // baseline: every node plain INPUT, pulls off, supply off
  enter(State::Prepare, now);
}

// Every node plain INPUT and the supply off first: nothing is powered
// while a pair is classified.
void beginClassification(uint8_t pairs, uint32_t now) {
  resetTestPins();
  classifyPending = pairs;
  enter(State::Classify, now);
  startNextPair(now);
}

void tickClassify(uint32_t now) {
  const WeakPullMeter::Result result = classifyMeter.advance();
  if (result == WeakPullMeter::Result::Pending) return;
  if (result == WeakPullMeter::Result::Done) {
    const int o = classifyReversed ? 1 : 0;
    classifyMv[o] = classifyMeter.meanMv();
    classifySpread[o] = classifyMeter.spreadMv();
    if (!classifyReversed) {
      if (startClassifyOrientation(true)) return;  // a setup failure leaves MeasurementError
    } else {
      evidence[classifyPair] = PairEvidence{classifyCouplingPair(classifyMv, classifySpread),
                                            {classifyMv[0], classifyMv[1]}};
    }
  }
  startNextPair(now);  // Failed keeps MeasurementError: never a 0 mV reading
}

// Kept for the Serial log when the decision involved coupling.
void recordEvaluation(const SensorDriver *driver, const ProbeMapping &m, const CouplingDecision &d) {
  CouplingEvaluation &e = status.evaluation;
  const DriverRequirements &r = driver->getRequirements();
  e.serial++;
  e.mapping = m;
  e.driverName = driver->getName();
  e.signalLabel[0] = r.signalLabel[0];
  e.signalLabel[1] = r.signalLabel[1];
  e.coupledPairs = status.coupledPairs;
  e.measuredPairs = measuredPairs;
  memcpy(e.pair, evidence, sizeof e.pair);
  e.verdict = d.verdict;
  memcpy(e.reason, d.reason, sizeof e.reason);
}

// ---- Discovery flow ------------------------------------------------------------

void startDiscovery(uint32_t now) {
  clearResult();
  clearCandidates();
  noComponent = false;
  attemptIndex = 0;
  twoTerminalHits = 0;
  cycleMeasurementError = false;
  status.measurementUnavailable = false;
  status.inconclusiveCount = 0;
  status.inconclusiveAttempt = 0;
  status.hardwareRequiredMask = hardwareRequiredMask();  // skipped, never a scan fault
  if (attemptTotal() == 0) {  // nothing to try: never precheck or excite
    noComponent = true;
    goIdle(now, PROBE_RESCAN_INTERVAL_MS);
    return;
  }
  beginPrecheck(now);
}

// Makes tentative candidate `index` the saved candidate. Like any saved
// mapping it is powered only after the power-off wait, a fresh precheck and
// the vetoes, and re-probed before anything is shown.
bool selectCandidate(uint8_t index, uint32_t now) {
  Attempt a;
  if (index >= status.candidateCount || !attemptAt(candidateAttempt[index], a)) return false;
  clearResult();
  saveCandidate(a.driver, *a.mapping);
  status.componentName = a.driver->getName();
  status.candidateSelected = static_cast<uint8_t>(index + 1);
  awaitingSelection = false;
  noComponent = false;
  enter(State::PowerOffWait, now);
  return true;
}

void finishCycle(uint32_t now) {
  Attempt hit;
  if (twoTerminalHits == 1 && attemptAt(twoTerminalHitIndex, hit)) {
    // Exactly one pair behaved as a two-terminal part: make it the
    // candidate and re-probe it from a fresh precheck (the scan result
    // itself is not reused as a confirming acquisition). A stable digital
    // level never overrides it; tentative candidates stay selectable.
    saveCandidate(hit.driver, *hit.mapping);
    status.componentName = hit.driver->getName();
    status.consecutiveFailures = 0;
    enter(State::PowerOffWait, now);
    return;
  }
  if (status.candidateCount == 1 && selectCandidate(0, now)) return;  // shown as tentative only
  noComponent = true;
  if (twoTerminalHits > 1) status.lastError = "ambiguous-pairs";  // multi-node coupling
  if (status.candidateCount > 1) {
    // Indistinguishable evidence: no mapping is picked, and the list is
    // kept (no rescan) until the user picks one or clears.
    status.lastError = "ambiguous-mappings";
    awaitingSelection = true;
  }
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
// capability, disabled topology, inconclusive precheck); nothing was
// energised.
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
  const bool pair = driver->getRequirements().topology == ProbeTopology::TwoTerminal;
  if (savedDriver == nullptr && (pair || driver->evidence() == EvidenceLevel::Candidate)) {
    // Two-terminal results are judged over every pair before one is
    // published, so a conducting pair is never reported while another
    // node is also coupled. Candidate evidence is judged over the whole
    // cycle, so it never stops discovery or hides a later protocol match.
    if (pair) {
      twoTerminalHits++;
      twoTerminalHitIndex = attemptIndex;
    } else if (status.candidateCount < PROBE_MAX_CANDIDATES) {
      candidateAttempt[status.candidateCount++] = static_cast<uint16_t>(attemptIndex);
    }
    if (!releaseAttempt(driver, currentMapping())) {
      status.lastError = "release-failed";
      enterFault(now);
      return;
    }
    advanceAttempt(now, true);
    return;
  }
  if (savedDriver == nullptr) {
    clearCandidates();  // a validated response supersedes every tentative one
    saveCandidate(driver, *currentMapping());  // stop scanning
  }
  status.componentName = driver->getName();
  status.evidence = driver->evidence();
  countConfirmation();
  // The mapping stays prepared; telemetry arrives with the next readSample().
  enter(status.detected ? State::Monitoring : State::Confirming, now);
}

void onSampleSucceeded(const char *line1, const char *line2, uint32_t now) {
  snprintf(status.telemetry1, sizeof status.telemetry1, "%s", line1);
  snprintf(status.telemetry2, sizeof status.telemetry2, "%s", line2);
  status.componentName = savedDriver->getName();  // classification may evolve
  status.evidence = savedDriver->evidence();       // e.g. a guided verification completed
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
  status.lastError = reason;
  if (!releaseAttempt(driver, currentMapping())) {
    // Power is already off, but the driver could not release a peripheral
    // it acquired. Never start another attempt with ownership uncertain.
    status.lastError = "release-failed";
    enterFault(now);
    return;
  }

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
      if (!awaitingSelection && elapsed(now, stateSinceMs, idleDelayMs)) startDiscovery(now);
      break;

    case State::Precheck:
      tickPrecheck(now);
      break;

    case State::Classify:
      tickClassify(now);
      break;

    case State::Prepare: {
      SensorDriver *driver = currentDriver();
      const ProbeMapping *m = currentMapping();
      if (driver == nullptr || m == nullptr) {
        enterFault(now);
        break;
      }
      const DriverRequirements &req = driver->getRequirements();
      // 1. Capabilities, enabled topology and an eligible supply profile.
      if (const char *blocked = attemptBlockReason(req, *m)) {
        onAttemptBlocked(now, blocked);
        break;
      }
      // 2-4. Coupling policy (WeakPullOnly: coupling is exactly what it measures).
      if (req.excitation == ExcitationPolicy::ProtectedActive) {
        const CouplingDecision d = decideCoupling(req, *m, status.coupledPairs, measuredPairs, evidence);
        if (d.verdict == CouplingVerdict::NeedsMeasurement) {
          beginClassification(d.measure, now);  // back here when done, still unpowered
          break;
        }
        if (d.verdict != CouplingVerdict::Allowed) recordEvaluation(driver, *m, d);
        if (d.verdict == CouplingVerdict::Blocked) {
          for (int i = 0; i < PROBE_PAIR_COUNT; i++) {
            if (evidence[i].classification == CouplingClassification::MeasurementError) cycleMeasurementError = true;
          }
          onAttemptBlocked(now, "blocked-by-precheck");
          break;
        }
      }
      // 5. The existing power-and-probe sequence.
      if (!prepareExcitation(req, *m)) {
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
         sameText(a.componentName, b.componentName) && a.evidence == b.evidence && a.detected == b.detected &&
         a.readingValid == b.readingValid && a.readingStale == b.readingStale &&
         sameText(a.telemetry1, b.telemetry1) && sameText(a.telemetry2, b.telemetry2) &&
         a.consecutiveFailures == b.consecutiveFailures && sameText(a.lastError, b.lastError) &&
         a.precheck == b.precheck && a.coupledPairs == b.coupledPairs &&
         a.measurementUnavailable == b.measurementUnavailable &&
         a.hardwareRequiredMask == b.hardwareRequiredMask && a.candidateCount == b.candidateCount &&
         a.candidateSelected == b.candidateSelected;
}

void publish(const ProbeStatus &before) {
  status.phase = derivePhase();
  status.scanTotal = static_cast<uint16_t>(attemptTotal());
  status.scanIndex = status.phase == ProbePhase::Scanning && attemptIndex < status.scanTotal
                         ? static_cast<uint16_t>(attemptIndex + 1)
                         : 0;
  if (!sameForDisplay(before, status)) status.revision++;
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
  classifyMeter.release();  // a pair classification in progress: sink and pull-up off
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
  awaitingSelection = false;
  cycleMeasurementError = false;
  status.hardwareRequiredMask = hardwareRequiredMask();

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
  if (releaseAttempt(currentDriver(), currentMapping())) {  // driver, then power and pins
    startDiscovery(millis());
  } else {
    status.lastError = "release-failed";
    enterFault(millis());
  }
  publish(before);
  return true;
}

bool probeSendCommand(char command) {
  if (savedDriver == nullptr || !status.detected) return false;
  return savedDriver->handleUserCommand(command);
}

bool probeSelectCandidate() {
  const bool discovering = savedDriver == nullptr && state != State::Idle;
  if (state == State::Disabled || state == State::Fault || discovering || status.candidateCount == 0) {
    return false;
  }
  const ProbeStatus before = status;
  const uint32_t now = millis();
  if (!releaseAttempt(currentDriver(), currentMapping())) {  // driver, then power and pins
    status.lastError = "release-failed";
    enterFault(now);
  } else if (!selectCandidate(static_cast<uint8_t>(status.candidateSelected % status.candidateCount), now)) {
    enterFault(now);  // the list no longer matches the attempt table: internal fault
  }
  publish(before);
  return true;
}

bool probeCandidateMapping(uint8_t index, ProbeMapping &out) {
  Attempt a;
  if (index >= status.candidateCount || !attemptAt(candidateAttempt[index], a)) return false;
  out = rolesFor(a.driver->getRequirements(), *a.mapping);
  return true;
}

bool probeFormatDiagnostics(char *buf, size_t len) {
  if (buf == nullptr || len == 0) return false;
  buf[0] = '\0';
  if (savedDriver == nullptr || !status.detected) return false;
  return savedDriver->formatDiagnostics(buf, len);
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
    appendNode(n < PROBE_NODE_COUNT / 2 ? first : second, len, n, probeRoleName(mapping.role[n], signalLabel));
  }
}

const char *probeRoleName(NodeRole role, const char *const *signalLabel) {
  switch (role) {
    case NodeRole::Vcc: return "VCC";
    case NodeRole::Data: return "DAT";
    case NodeRole::Out: return "OUT";
    case NodeRole::Gnd: return "GND";
    case NodeRole::Terminal: return "T";
    case NodeRole::Sig1: return signalLabel != nullptr && signalLabel[0] != nullptr ? signalLabel[0] : "S1";
    case NodeRole::Sig2: return signalLabel != nullptr && signalLabel[1] != nullptr ? signalLabel[1] : "S2";
    case NodeRole::Unused: break;
  }
  return "---";
}

// ---- Coupling policy -------------------------------------------------------------

static_assert(PASSIVE_CONTACT_MAX_MV < PULLUP_COMPATIBLE_MIN_MV &&
                  PULLUP_COMPATIBLE_MIN_MV <= PULLUP_COMPATIBLE_MAX_MV &&
                  PULLUP_COMPATIBLE_MAX_MV < PASSIVE_OPEN_MIN_MV,
              "the pull-up band must sit above the contact band and below the open band");
static_assert(PULLUP_SERIES_TOLERANCE_PCT > 0 && PULLUP_SERIES_TOLERANCE_PCT < 100,
              "series tolerance is a percentage");

CouplingClassification classifyCouplingPair(const uint16_t mv[2], const uint16_t spreadMv[2]) {
  // Contact-like in either direction: if it conducts that well one way,
  // powered drive could push current that way. Never a pull-up.
  if (mv[0] <= PASSIVE_CONTACT_MAX_MV || mv[1] <= PASSIVE_CONTACT_MAX_MV) {
    return CouplingClassification::LowImpedance;
  }
  if (spreadMv[0] > PASSIVE_STABLE_SPREAD_MV || spreadMv[1] > PASSIVE_STABLE_SPREAD_MV) {
    return CouplingClassification::Ambiguous;  // unstable (noise, charging capacitance)
  }
  const uint32_t mean = (static_cast<uint32_t>(mv[0]) + mv[1]) / 2;
  const uint32_t pct = mean * PASSIVE_SYMMETRY_PCT / 100;
  const uint32_t tolerance = pct > PASSIVE_SYMMETRY_MIN_DELTA_MV ? pct : PASSIVE_SYMMETRY_MIN_DELTA_MV;
  const uint32_t delta = mv[0] > mv[1] ? mv[0] - mv[1] : mv[1] - mv[0];
  if (delta > tolerance) return CouplingClassification::Ambiguous;  // directional: junctions, regulators
  if (mean >= PASSIVE_OPEN_MIN_MV) return CouplingClassification::OpenLike;  // no finite path seen
  if (mean >= PULLUP_COMPATIBLE_MIN_MV && mean <= PULLUP_COMPATIBLE_MAX_MV) {
    return CouplingClassification::PullupCompatible;
  }
  return CouplingClassification::Resistive;
}

bool pullupSeriesExplained(uint16_t vccSig1Mv, uint16_t vccSig2Mv, uint16_t sig1Sig2Mv) {
  uint32_t r1 = 0, r2 = 0, r12 = 0;
  if (!passiveEstimateOhms(vccSig1Mv, r1) || !passiveEstimateOhms(vccSig2Mv, r2) ||
      !passiveEstimateOhms(sig1Sig2Mv, r12)) {
    return false;  // a bound, not a value: nothing to compare
  }
  // The estimates share the nominal pull-up, so its absolute error cancels
  // in this comparison; pin-to-pin mismatch and ADC error remain.
  const uint32_t sum = r1 + r2;
  const uint32_t tolerance = sum * PULLUP_SERIES_TOLERANCE_PCT / 100;
  return r12 + tolerance >= sum && r12 <= sum + tolerance;
}

// A Blocked decision with a formatted (compiler-checked) reason.
static CouplingDecision blockedBy(const char *format, ...) __attribute__((format(printf, 1, 2)));
static CouplingDecision blockedBy(const char *format, ...) {
  CouplingDecision d{CouplingVerdict::Blocked, 0, ""};
  va_list args;
  va_start(args, format);
  vsnprintf(d.reason, sizeof d.reason, format, args);
  va_end(args);
  return d;
}

CouplingDecision decideCoupling(const DriverRequirements &r, const ProbeMapping &m, uint8_t coupled,
                                uint8_t measured, const PairEvidence pairEvidence[]) {
  const int vcc = probeNodeWithRole(m, NodeRole::Vcc);
  const int gnd = probeNodeWithRole(m, NodeRole::Gnd);
  if (vcc < 0 || gnd < 0) return blockedBy("no supply mapping");

  // The rail: VCC-GND coupling is vetoed for every powered candidate and
  // is never reclassified, whatever another pair looks like.
  if ((coupled & probePairBit(static_cast<uint8_t>(vcc), static_cast<uint8_t>(gnd))) != 0) {
    return blockedBy("VCC-GND coupling");
  }
  // The existing rule: nothing coupled to a driven path.
  if (drivenPathsUncoupled(m, coupled, r.openDrainSignals)) return CouplingDecision{CouplingVerdict::Allowed, 0, ""};
  if (!pullupExceptionEligible(r)) {
    return blockedBy(!drivenPathsUncoupled(m, coupled, true) ? "VCC coupling" : "push-pull signal coupling");
  }

  // Eligible, and its VCC is coupled to SIG1 and/or SIG2 only (four-wire
  // uses every node; digital VCC-GND coupling was vetoed above). Pull-ups
  // go to VCC, so a coupled signal-GND pair is not explained by them.
  const char *l1 = probeRoleName(NodeRole::Sig1, r.signalLabel);
  const char *l2 = probeRoleName(NodeRole::Sig2, r.signalLabel);
  const int s1 = probeNodeWithRole(m, NodeRole::Sig1);
  const int s2 = probeNodeWithRole(m, NodeRole::Sig2);
  const int rail = pairIndex(vcc, gnd);
  const int a = pairIndex(vcc, s1);
  const int b = pairIndex(vcc, s2);
  const int ab = pairIndex(s1, s2);
  const int ga = pairIndex(s1, gnd);
  const int gb = pairIndex(s2, gnd);
  if (rail < 0 || a < 0 || b < 0 || ab < 0 || ga < 0 || gb < 0) return blockedBy("no signal mapping");
  if ((coupled & ((1u << ga) | (1u << gb))) != 0) return blockedBy("signal-GND coupling unexplained");

  // The rail is measured too: the pull-ups load the precheck's weak
  // pull-down nodes, so a VCC-GND short can read LOW (uncoupled) there.
  const uint8_t needed = static_cast<uint8_t>((1u << rail) | (1u << a) | (1u << b) | (1u << ab));
  if ((measured & needed) != needed) {
    return CouplingDecision{CouplingVerdict::NeedsMeasurement, static_cast<uint8_t>(needed & ~measured), ""};
  }

  const CouplingClassification cr = pairEvidence[rail].classification;
  const CouplingClassification ca = pairEvidence[a].classification;
  const CouplingClassification cb = pairEvidence[b].classification;
  const CouplingClassification cab = pairEvidence[ab].classification;
  if (cr == CouplingClassification::MeasurementError || ca == CouplingClassification::MeasurementError ||
      cb == CouplingClassification::MeasurementError || cab == CouplingClassification::MeasurementError) {
    return blockedBy("ADC measurement unavailable");
  }
  // Rail: only open, or finite and well above a short, in both directions.
  if (cr == CouplingClassification::LowImpedance) return blockedBy("VCC-GND low impedance");
  if (cr == CouplingClassification::Ambiguous || cr == CouplingClassification::Unclassified) {
    return blockedBy("VCC-GND evidence ambiguous");
  }
  if (ca == CouplingClassification::LowImpedance) return blockedBy("VCC-%s low impedance", l1);
  if (cb == CouplingClassification::LowImpedance) return blockedBy("VCC-%s low impedance", l2);
  if (cab == CouplingClassification::LowImpedance) return blockedBy("%s-%s low impedance", l1, l2);
  if (ca != CouplingClassification::PullupCompatible) {
    return blockedBy(ca == CouplingClassification::Resistive ? "VCC-%s outside pull-up range"
                                                             : "pull-up classification ambiguous (VCC-%s)",
                     l1);
  }
  if (cb != CouplingClassification::PullupCompatible) {
    return blockedBy(cb == CouplingClassification::Resistive ? "VCC-%s outside pull-up range"
                                                             : "pull-up classification ambiguous (VCC-%s)",
                     l2);
  }
  // Two pull-ups to one VCC also join SIG1 and SIG2 through each other; a
  // signal-to-signal path that does not match them in series (a direct
  // short, another network) is not explained.
  auto mean = [pairEvidence](int i) {
    return static_cast<uint16_t>((pairEvidence[i].mv[0] + pairEvidence[i].mv[1]) / 2);
  };
  const bool finite = cab == CouplingClassification::PullupCompatible || cab == CouplingClassification::Resistive;
  if (!finite || !pullupSeriesExplained(mean(a), mean(b), mean(ab))) {
    return blockedBy("%s-%s path not explained by pull-ups", l1, l2);
  }
  return CouplingDecision{CouplingVerdict::PullupException, 0, "open-drain pull-up exception"};
}

const char *couplingClassificationName(CouplingClassification c) {
  switch (c) {
    case CouplingClassification::Unclassified: return "unclassified";
    case CouplingClassification::PullupCompatible: return "pullup-compatible";
    case CouplingClassification::Resistive: return "resistive";
    case CouplingClassification::LowImpedance: return "low-impedance";
    case CouplingClassification::OpenLike: return "open-like";
    case CouplingClassification::Ambiguous: return "ambiguous";
    case CouplingClassification::MeasurementError: return "measurement-error";
  }
  return "unknown";
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
