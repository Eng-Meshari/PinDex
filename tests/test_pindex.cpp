// Host tests for PinDex. Not part of the Arduino build: the IDE compiles
// only the sketch folder root and src/, never tests/.
// Build from the PinDex folder with any C++17 host compiler, three ways:
//   SRC="tests/test_pindex.cpp probe_engine.cpp dht11.cpp
//        src/drivers/dht11_driver.cpp src/drivers/passive_driver.cpp
//        src/drivers/ultrasonic_driver.cpp"
//   g++ -std=c++17 -Wall -Wextra -Itests/mock -I. -DPINDEX_POWERED_PROBING_ENABLED=1
//       -DPINDEX_PASSIVE_PROBING_ENABLED=1 $SRC -o build/test_all
//   ...=0 / ...PASSIVE...=1 -> build/test_passive_only
//   ...=0 / ...PASSIVE...=0 -> build/test_disabled
// Run all three; each exits non-zero if any check fails. This file supplies
// the driver registry, the passive ADC adapter and the four-wire backend
// itself (link seams), so it is linked WITHOUT
// src/drivers/driver_registry.cpp, src/drivers/passive_adc.cpp and
// four_wire_backend.cpp. Shorts, resistors, faults, sensors and the
// protected four-wire backend are simulated; nothing here touches
// hardware, and no test drives opposing GPIO outputs into each other.

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <initializer_list>

#include <Arduino.h>  // tests/mock/Arduino.h

#include "config.h"
#include "dht11.h"
#include "four_wire_backend.h"
#include "probe_engine.h"
#include "sensor_driver.h"
#include "src/drivers/dht11_driver.h"
#include "src/drivers/driver_registry.h"
#include "src/drivers/passive_adc.h"
#include "src/drivers/passive_driver.h"
#include "src/drivers/ultrasonic_driver.h"

#define FULL_BUILD (PINDEX_POWERED_PROBING_ENABLED && PINDEX_PASSIVE_PROBING_ENABLED)
#define PASSIVE_ONLY_BUILD (!PINDEX_POWERED_PROBING_ENABLED && PINDEX_PASSIVE_PROBING_ENABLED)

static int checks = 0;
static int failures = 0;
#define CHECK(cond)                                           \
  do {                                                        \
    checks++;                                                 \
    if (!(cond)) {                                            \
      failures++;                                             \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);  \
    }                                                         \
  } while (0)

[[maybe_unused]] static bool same(const char *a, const char *b) {
  return a != nullptr && b != nullptr && strcmp(a, b) == 0;
}

// ---- Test registry (replaces src/drivers/driver_registry.cpp) --------------

static SensorDriver *testDrivers[4] = {};
static size_t testDriverCount = 0;

size_t sensorDriverCount() { return testDriverCount; }

SensorDriver *sensorDriverAt(size_t index) {
  return index < testDriverCount ? testDrivers[index] : nullptr;
}

static void useDrivers(SensorDriver *a = nullptr, SensorDriver *b = nullptr,
                       SensorDriver *c = nullptr, SensorDriver *d = nullptr) {
  testDriverCount = 0;
  for (SensorDriver *x : {a, b, c, d}) {
    if (x != nullptr) testDrivers[testDriverCount++] = x;
  }
}

// ---- Pure decoding ---------------------------------------------------------

static DhtPulses pulsesFor(const uint8_t bytes[DHT_FRAME_BYTES]) {
  DhtPulses p{};
  p.ackLowUs = 80;
  p.ackHighUs = 80;
  for (int i = 0; i < DHT_FRAME_BITS; i++) {
    p.bitLowUs[i] = 50;
    p.bitHighUs[i] = ((bytes[i / 8] >> (7 - i % 8)) & 1) ? 70 : 27;
  }
  return p;
}

static void testFrameDecoding() {
  DhtReading r{};
  const uint8_t valid[5] = {60, 0, 25, 0, 85};
  CHECK(dhtDecodeFrame(valid, r) == DhtError::None);
  CHECK(r.humidityPct == 60 && r.temperatureDeciC == 250);

  const uint8_t tenths[5] = {55, 0, 23, 7, 85};
  CHECK(dhtDecodeFrame(tenths, r) == DhtError::None);
  CHECK(r.temperatureDeciC == 237);

  // The checksum is modulo 256 and independent of range/encoding checks.
  const uint8_t wraps[5] = {200, 100, 0, 0, 44};  // 300 mod 256 = 44
  CHECK(dhtChecksumOk(wraps));
  CHECK(dhtDecodeFrame(wraps, r) == DhtError::UnsupportedEncoding);
  const uint8_t wrapsRange[5] = {250, 0, 10, 0, 4};  // 260 mod 256 = 4
  CHECK(dhtChecksumOk(wrapsRange));
  CHECK(dhtDecodeFrame(wrapsRange, r) == DhtError::OutOfRange);

  const uint8_t badSum[5] = {60, 0, 25, 0, 86};
  CHECK(!dhtChecksumOk(badSum));
  CHECK(dhtDecodeFrame(badSum, r) == DhtError::Checksum);

  // Other encodings are rejected, not reinterpreted.
  const uint8_t humDecimal[5] = {60, 5, 25, 0, 90};
  CHECK(dhtDecodeFrame(humDecimal, r) == DhtError::UnsupportedEncoding);
  const uint8_t negative[5] = {60, 0, 2, 0x81, static_cast<uint8_t>(60 + 2 + 0x81)};
  CHECK(dhtDecodeFrame(negative, r) == DhtError::UnsupportedEncoding);
  const uint8_t dht22[5] = {0x02, 0x8C, 0x00, 0xFD, static_cast<uint8_t>(0x02 + 0x8C + 0xFD)};
  CHECK(dhtDecodeFrame(dht22, r) == DhtError::UnsupportedEncoding);

  // Plausibility window of the supported profile.
  const uint8_t hot[5] = {60, 0, 51, 0, 111};
  CHECK(dhtDecodeFrame(hot, r) == DhtError::OutOfRange);
  const uint8_t dry[5] = {3, 0, 25, 0, 28};
  CHECK(dhtDecodeFrame(dry, r) == DhtError::OutOfRange);
  const uint8_t zeros[5] = {0, 0, 0, 0, 0};  // checksum passes, still rejected
  CHECK(dhtChecksumOk(zeros));
  CHECK(dhtDecodeFrame(zeros, r) == DhtError::OutOfRange);
}

static void testPulseDecoding() {
  static const uint8_t kZero[5] = {0, 0, 0, 0, 0};
  const uint8_t frame[5] = {60, 0, 25, 0, 85};
  uint8_t bytes[5];

  DhtPulses p = pulsesFor(frame);
  CHECK(dhtPulsesToBytes(p, bytes) == DhtError::None);
  CHECK(memcmp(bytes, frame, 5) == 0);

  // The threshold decides 0 vs 1; bit 0 is the MSB of byte 0.
  p.bitHighUs[0] = DHT_BIT_THRESHOLD_US - 1;
  CHECK(dhtPulsesToBytes(p, bytes) == DhtError::None && (bytes[0] & 0x80) == 0);
  p.bitHighUs[0] = DHT_BIT_THRESHOLD_US + 1;
  CHECK(dhtPulsesToBytes(p, bytes) == DhtError::None && (bytes[0] & 0x80) != 0);

  p = pulsesFor(frame);
  p.ackLowUs = 20;
  CHECK(dhtPulsesToBytes(p, bytes) == DhtError::BadAckTiming);
  CHECK(memcmp(bytes, kZero, 5) == 0);

  p = pulsesFor(frame);
  p.bitHighUs[17] = 105;
  CHECK(dhtPulsesToBytes(p, bytes) == DhtError::BadBitTiming);
  CHECK(memcmp(bytes, kZero, 5) == 0);  // no partial data

  p = pulsesFor(frame);
  p.bitLowUs[3] = 10;
  CHECK(dhtPulsesToBytes(p, bytes) == DhtError::BadBitTiming);

  // A zeroed (incomplete) tail never decodes.
  p = pulsesFor(frame);
  for (int i = 20; i < DHT_FRAME_BITS; i++) p.bitLowUs[i] = p.bitHighUs[i] = 0;
  CHECK(dhtPulsesToBytes(p, bytes) == DhtError::BadBitTiming);
}

// Mappings in `table` that repeat an earlier one.
static int duplicates(const ProbeMapping *table, int count) {
  int dup = 0;
  for (int i = 0; i < count; i++) {
    for (int j = 0; j < i; j++) {
      if (memcmp(table[i].role, table[j].role, sizeof table[i].role) == 0) dup++;
    }
  }
  return dup;
}

static int roleCount(const ProbeMapping &m, NodeRole role) {
  int count = 0;
  for (NodeRole r : m.role) count += r == role ? 1 : 0;
  return count;
}

static void testMappingTables() {
  CHECK(PROBE_NODE_COUNT == 4);
  CHECK(PROBE_NODE_GPIO[0] == 25 && PROBE_NODE_GPIO[1] == 26 && PROBE_NODE_GPIO[2] == 27 &&
        PROBE_NODE_GPIO[3] == 33);
  CHECK(PROBE_NODE_LABEL[0] == 'A' && PROBE_NODE_LABEL[3] == 'D');

  // Three-wire: 24 distinct ordered triples, each with one node unused.
  CHECK(duplicates(PROBE_PERMUTATIONS, PROBE_PERMUTATION_COUNT) == 0);
  for (const ProbeMapping &m : PROBE_PERMUTATIONS) {
    CHECK(m.topology == ProbeTopology::ThreeWirePowered);
    CHECK(roleCount(m, NodeRole::Vcc) == 1 && roleCount(m, NodeRole::Data) == 1 &&
          roleCount(m, NodeRole::Gnd) == 1 && roleCount(m, NodeRole::Unused) == 1);
  }
  // The original six A/B/C permutations come first, in their Phase 2 order
  // ({VCC, DATA, GND} within the node set), then the same patterns on ABD,
  // ACD and BCD.
  const int pattern[6][3] = {{0, 1, 2}, {2, 1, 0}, {1, 0, 2}, {1, 2, 0}, {0, 2, 1}, {2, 0, 1}};
  const int sets[4][3] = {{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};
  for (int i = 0; i < PROBE_PERMUTATION_COUNT; i++) {
    const ProbeMapping &m = PROBE_PERMUTATIONS[i];
    const int *set = sets[i / 6];
    const int *p = pattern[i % 6];
    CHECK(probeNodeWithRole(m, NodeRole::Vcc) == set[p[0]] &&
          probeNodeWithRole(m, NodeRole::Data) == set[p[1]] &&
          probeNodeWithRole(m, NodeRole::Gnd) == set[p[2]]);
    if (i < 6) CHECK(m.role[3] == NodeRole::Unused);  // D untouched by the original scan
  }

  // Four-wire: 24 distinct orderings, every node used once.
  CHECK(duplicates(PROBE_FOUR_WIRE_MAPPINGS, PROBE_FOUR_WIRE_COUNT) == 0);
  for (const ProbeMapping &m : PROBE_FOUR_WIRE_MAPPINGS) {
    CHECK(m.topology == ProbeTopology::FourWirePowered);
    CHECK(roleCount(m, NodeRole::Vcc) == 1 && roleCount(m, NodeRole::Sig1) == 1 &&
          roleCount(m, NodeRole::Sig2) == 1 && roleCount(m, NodeRole::Gnd) == 1 &&
          roleCount(m, NodeRole::Unused) == 0);
  }
  const ProbeMapping &first = PROBE_FOUR_WIRE_MAPPINGS[0];
  const ProbeMapping &last = PROBE_FOUR_WIRE_MAPPINGS[PROBE_FOUR_WIRE_COUNT - 1];
  CHECK(first.role[0] == NodeRole::Vcc && first.role[1] == NodeRole::Sig1 &&
        first.role[2] == NodeRole::Sig2 && first.role[3] == NodeRole::Gnd);
  CHECK(last.role[3] == NodeRole::Vcc && last.role[2] == NodeRole::Sig1 &&
        last.role[1] == NodeRole::Sig2 && last.role[0] == NodeRole::Gnd);

  // Pairs: six distinct unordered pairs (a pair's two orientations are one
  // physical pair), two nodes unused; the original three first.
  CHECK(duplicates(PROBE_PAIRS, PROBE_PAIR_COUNT) == 0);
  const int pairNodes[PROBE_PAIR_COUNT][2] = {{0, 1}, {1, 2}, {0, 2}, {0, 3}, {1, 3}, {2, 3}};
  for (int i = 0; i < PROBE_PAIR_COUNT; i++) {
    const ProbeMapping &m = PROBE_PAIRS[i];
    CHECK(m.topology == ProbeTopology::TwoTerminal);
    CHECK(roleCount(m, NodeRole::Terminal) == 2 && roleCount(m, NodeRole::Unused) == 2);
    CHECK(probeNodeWithRole(m, NodeRole::Terminal, 0) == pairNodes[i][0] &&
          probeNodeWithRole(m, NodeRole::Terminal, 1) == pairNodes[i][1]);
  }

  // One coupling bit per pair, symmetric, covering all six (D included).
  uint8_t all = 0;
  for (uint8_t x = 0; x < PROBE_NODE_COUNT; x++) {
    for (uint8_t y = 0; y < PROBE_NODE_COUNT; y++) {
      if (x == y) continue;
      CHECK(probePairBit(x, y) != 0 && probePairBit(x, y) == probePairBit(y, x));
      all |= probePairBit(x, y);
    }
  }
  CHECK(all == (1u << PROBE_PAIR_COUNT) - 1);
  CHECK(probePairBit(0, 1) == 1 && probePairBit(1, 2) == 2 && probePairBit(0, 2) == 4);
  CHECK(probePairBit(0, 3) == 8 && probePairBit(1, 3) == 16 && probePairBit(2, 3) == 32);
  CHECK(probePairBit(1, 1) == 0 && probePairBit(0, PROBE_NODE_COUNT) == 0);
}

static void testMappingText() {
  static const char *const kLabels[2] = {"TRG", "ECH"};
  struct Case {
    const ProbeMapping *mapping;
    const char *const *labels;
    const char *first;
    const char *second;
  };
  const Case cases[] = {
      {&PROBE_PERMUTATIONS[0], nullptr, "A:VCC B:DAT", "C:GND D:---"},
      {&PROBE_PERMUTATIONS[3], nullptr, "A:GND B:VCC", "C:DAT D:---"},
      {&PROBE_PERMUTATIONS[6], nullptr, "A:VCC B:DAT", "C:--- D:GND"},
      {&PROBE_PERMUTATIONS[19], nullptr, "A:--- B:GND", "C:DAT D:VCC"},
      {&PROBE_FOUR_WIRE_MAPPINGS[0], kLabels, "A:VCC B:TRG", "C:ECH D:GND"},
      {&PROBE_FOUR_WIRE_MAPPINGS[23], kLabels, "A:GND B:ECH", "C:TRG D:VCC"},
      {&PROBE_FOUR_WIRE_MAPPINGS[0], nullptr, "A:VCC B:S1", "C:S2 D:GND"},
      {&PROBE_PAIRS[0], nullptr, "Pins: A & B", "C:--- D:---"},
      {&PROBE_PAIRS[2], nullptr, "Pins: A & C", "B:--- D:---"},
      {&PROBE_PAIRS[5], nullptr, "Pins: C & D", "A:--- B:---"},
  };
  char a[PROBE_MAPPING_TEXT_LEN];
  char b[PROBE_MAPPING_TEXT_LEN];
  for (const Case &c : cases) {
    formatProbeMapping(*c.mapping, c.labels, a, b, sizeof a);
    CHECK(same(a, c.first) && same(b, c.second));
    CHECK(strlen(a) * 6 <= OLED_WIDTH && strlen(b) * 6 <= OLED_WIDTH);  // size-1 font: 6 px/glyph
  }
  // The joined four-node line ("A:VCC B:TRG C:ECH D:GND", 138 px) does not
  // fit at the existing font, which is why the renderer measures and splits.
  formatProbeMapping(PROBE_FOUR_WIRE_MAPPINGS[0], kLabels, a, b, sizeof a);
  CHECK((strlen(a) + 1 + strlen(b)) * 6 > OLED_WIDTH);

  // Small buffers truncate but stay terminated.
  char s1[5];
  char s2[5];
  formatProbeMapping(PROBE_FOUR_WIRE_MAPPINGS[0], kLabels, s1, s2, sizeof s1);
  CHECK(same(s1, "A:VC") && same(s2, "C:EC"));
}

static void testPassiveEstimate() {
  uint32_t ohms = 0;
  CHECK(!passiveEstimateOhms(PASSIVE_CONTACT_MAX_MV - 1, ohms));  // below the useful window
  CHECK(!passiveEstimateOhms(PASSIVE_OPEN_MIN_MV + 1, ohms));     // above it
  CHECK(!passiveEstimateOhms(PASSIVE_SUPPLY_MV, ohms));           // zero denominator
  CHECK(passiveEstimateOhms(600, ohms) && ohms == 10000);
  CHECK(passiveEstimateOhms(1650, ohms) && ohms == 45000);
  CHECK(passiveEstimateOhms(PASSIVE_CONTACT_MAX_MV, ohms) && ohms == 2142);
  CHECK(passiveEstimateOhms(PASSIVE_OPEN_MIN_MV, ohms) && ohms == 129705);
}

// ---- Shared simulation helpers ---------------------------------------------

static int lockLeaks = 0;
static int lockedPinModes = 0;
static int highDriveEvents = 0;  // any probe pin driven HIGH (push-pull)
static int adcReadsInCritical = 0;
static int adcWithIsrAttached = 0;  // passive sensing began with a probe interrupt still attached
static bool flakyA = false;  // A alternates HIGH/LOW while pulled up
static int flakyReads = 0;

// Operation starts logged by FakeDriver.
struct Attempt {
  int driver;
  int slot;  // index into the mapping table of its topology
  bool sample;
  ProbeTopology topology;
  uint64_t us;
};
static Attempt attempts[256];
static int attemptCount = 0;

static uint8_t gpioOf(int node) { return PROBE_NODE_GPIO[node]; }

static int nodeOf(uint8_t pin) {
  for (int n = 0; n < PROBE_NODE_COUNT; n++) {
    if (PROBE_NODE_GPIO[n] == pin) return n;
  }
  return -1;
}

static bool allProbesPlainInput() {
  for (uint8_t g : PROBE_NODE_GPIO) {
    if (mock::pins[g].mode != INPUT) return false;
  }
  return true;
}

static bool anyProbeIsrAttached() {
  for (uint8_t g : PROBE_NODE_GPIO) {
    if (mock::isr[g].fn != nullptr) return true;
  }
  return false;
}

// The context the engine hands a driver for mapping m.
static ProbeContext contextOf(const ProbeMapping &m) {
  auto pin = [&m](NodeRole role, int nth) {
    const int n = probeNodeWithRole(m, role, nth);
    return n >= 0 ? gpioOf(n) : static_cast<uint8_t>(0xFF);
  };
  ProbeContext c{};
  c.topology = m.topology;
  switch (m.topology) {
    case ProbeTopology::ThreeWirePowered:
      c.powered = {pin(NodeRole::Vcc, 0), pin(NodeRole::Data, 0), pin(NodeRole::Gnd, 0)};
      break;
    case ProbeTopology::FourWirePowered:
      c.fourWire = {pin(NodeRole::Vcc, 0), pin(NodeRole::Sig1, 0), pin(NodeRole::Sig2, 0),
                    pin(NodeRole::Gnd, 0)};
      break;
    case ProbeTopology::TwoTerminal:
      c.pair = {pin(NodeRole::Terminal, 0), pin(NodeRole::Terminal, 1),
                {pin(NodeRole::Unused, 0), pin(NodeRole::Unused, 1)}};
      break;
  }
  return c;
}

static bool sameContext(const ProbeContext &a, const ProbeContext &b) {
  if (a.topology != b.topology) return false;
  switch (a.topology) {
    case ProbeTopology::ThreeWirePowered: return memcmp(&a.powered, &b.powered, sizeof a.powered) == 0;
    case ProbeTopology::FourWirePowered: return memcmp(&a.fourWire, &b.fourWire, sizeof a.fourWire) == 0;
    case ProbeTopology::TwoTerminal: return memcmp(&a.pair, &b.pair, sizeof a.pair) == 0;
  }
  return false;
}

// Index of the context's mapping in its topology's table, or -1.
static int slotOf(const ProbeContext &c) {
  const ProbeMapping *table = PROBE_PAIRS;
  int count = PROBE_PAIR_COUNT;
  if (c.topology == ProbeTopology::ThreeWirePowered) {
    table = PROBE_PERMUTATIONS;
    count = PROBE_PERMUTATION_COUNT;
  } else if (c.topology == ProbeTopology::FourWirePowered) {
    table = PROBE_FOUR_WIRE_MAPPINGS;
    count = PROBE_FOUR_WIRE_COUNT;
  }
  for (int i = 0; i < count; i++) {
    if (table[i].topology == c.topology && sameContext(contextOf(table[i]), c)) return i;
  }
  return -1;
}

[[maybe_unused]] static ProbeContext poweredContext(int perm) { return contextOf(PROBE_PERMUTATIONS[perm]); }
[[maybe_unused]] static ProbeContext pairContext(int pair) { return contextOf(PROBE_PAIRS[pair]); }
[[maybe_unused]] static ProbeContext fourWireContext(int i) { return contextOf(PROBE_FOUR_WIRE_MAPPINGS[i]); }

[[maybe_unused]] static bool sameRoles(const ProbeMapping &a, const ProbeMapping &b) {
  return a.topology == b.topology && memcmp(a.role, b.role, sizeof a.role) == 0;
}

static void advanceTo(uint64_t untilUs);  // delivers simulated ECHO edges (below)

// One loop() iteration per simulated millisecond.
static void tick() {
  advanceTo(mock::nowUs + 1000);
  probeTick();
  if (mock::criticalDepth != 0) lockLeaks++;
}

static void runMs(uint32_t ms) {
  for (uint32_t i = 0; i < ms; i++) tick();
}

template <typename Done>
static bool runUntil(Done done, uint32_t maxMs) {
  for (uint32_t i = 0; i < maxMs && !done(); i++) tick();
  return done();
}

static ProbePhase phase() { return probeStatus().phase; }

// Waits for n newly published samples.
[[maybe_unused]] static bool waitSamples(int n) {
  for (int i = 0; i < n; i++) {
    const uint32_t last = probeStatus().lastReadingMs;
    if (!runUntil([last] { return probeStatus().lastReadingMs != last; }, 1000)) return false;
  }
  return true;
}

// Drives a driver directly (no engine), one simulated ms per call.
[[maybe_unused]] static bool driveProbe(SensorDriver &d, const ProbeContext &c) {
  for (int i = 0; i < 1000; i++) {
    if (d.probe(c)) return true;
    if (d.getOperationStatus() != DriverOperationStatus::Pending) return false;
    advanceTo(mock::nowUs + 1000);
  }
  return false;
}

[[maybe_unused]] static bool driveSample(SensorDriver &d, char *l1, char *l2, size_t len) {
  for (int i = 0; i < 1000; i++) {
    if (d.readSample(l1, l2, len)) return true;
    if (d.getOperationStatus() != DriverOperationStatus::Pending) return false;
    advanceTo(mock::nowUs + 1000);
  }
  return false;
}

// ---- Resistor-network simulation -------------------------------------------
// Node voltages from the pin configuration (pulls 45 kOhm, outputs 25 Ohm)
// plus simulated components between nodes. Drives digitalRead() (precheck)
// and the fake ADC, so both see the same circuit.

const double kOpenOhms = 1e12;

struct Network {
  double r[PROBE_NODE_COUNT][PROBE_NODE_COUNT];
  int asymSenseNode = -1;  // directional element: reads asymMv while a sink is on
  uint16_t asymMv = 0;
  uint16_t noiseMv = 0;    // alternating +/- on every ADC reading
  bool adcUnavailable = false;
  bool adcBeginFails = false;
  bool adcReadFails = false;
  bool pullupIneffective = false;  // pull-up has no effect on an analog pad
};
static Network net;
static bool noiseFlip = false;

static void netReset() {
  net = Network();
  for (auto &row : net.r) {
    for (double &x : row) x = kOpenOhms;
  }
  noiseFlip = false;
}

[[maybe_unused]] static void connect(int x, int y, double ohms) {
  net.r[x][y] = ohms;
  net.r[y][x] = ohms;
}

static void solveNodes(double v[PROBE_NODE_COUNT]) {
  double g[PROBE_NODE_COUNT][PROBE_NODE_COUNT + 1] = {};
  for (int i = 0; i < PROBE_NODE_COUNT; i++) {
    const mock::Pin &p = mock::pins[PROBE_NODE_GPIO[i]];
    auto toRail = [&](double conductance, double volts) {
      g[i][i] += conductance;
      g[i][PROBE_NODE_COUNT] += conductance * volts;
    };
    toRail(1e-12, 0.0);  // leakage: a floating node settles at 0 V
    const bool output = (p.mode & OUTPUT) == OUTPUT;
    if (output && !(p.mode & OPEN_DRAIN)) toRail(1.0 / 25.0, p.latch ? 3.3 : 0.0);
    if (output && (p.mode & OPEN_DRAIN) && p.latch == LOW) toRail(1.0 / 25.0, 0.0);
    const bool analog = (p.mode & ANALOG) == ANALOG;
    if ((p.mode & PULLUP) && !(analog && net.pullupIneffective)) toRail(1.0 / 45000.0, 3.3);
    if (!analog && (p.mode & PULLDOWN)) toRail(1.0 / 45000.0, 0.0);
    for (int j = 0; j < PROBE_NODE_COUNT; j++) {
      if (j == i) continue;
      const double c = 1.0 / net.r[i][j];
      g[i][i] += c;
      g[i][j] -= c;
    }
  }
  for (int c = 0; c < PROBE_NODE_COUNT; c++) {  // Gauss-Jordan with pivoting
    int pivot = c;
    for (int r = c + 1; r < PROBE_NODE_COUNT; r++) {
      if (fabs(g[r][c]) > fabs(g[pivot][c])) pivot = r;
    }
    for (int k = 0; k <= PROBE_NODE_COUNT; k++) {
      const double t = g[c][k];
      g[c][k] = g[pivot][k];
      g[pivot][k] = t;
    }
    for (int r = 0; r < PROBE_NODE_COUNT; r++) {
      if (r == c) continue;
      const double f = g[r][c] / g[c][c];
      for (int k = c; k <= PROBE_NODE_COUNT; k++) g[r][k] -= f * g[c][k];
    }
  }
  for (int i = 0; i < PROBE_NODE_COUNT; i++) v[i] = g[i][PROBE_NODE_COUNT] / g[i][i];
}

static bool sinkEngaged() {
  for (uint8_t g : PROBE_NODE_GPIO) {
    if (mock::isOutput(g) && (mock::pins[g].mode & OPEN_DRAIN) && mock::pins[g].latch == LOW) {
      return true;
    }
  }
  return false;
}

// ---- Fake passive ADC adapter (replaces src/drivers/passive_adc.cpp) --------

static uint8_t senseLog[128];
static int senseCount = 0;

PassiveAdcStatus passiveAdcBeginSense(uint8_t pin) {
  if (senseCount < 128) senseLog[senseCount++] = pin;
  if (anyProbeIsrAttached()) adcWithIsrAttached++;  // an ultrasonic interrupt was not released
  if (net.adcUnavailable) return PassiveAdcStatus::Unavailable;
  if (net.adcBeginFails) return PassiveAdcStatus::ConfigFailed;
  mock::pins[pin].mode = ANALOG | PULLUP;  // analog pad, pull-up re-enabled after setup
  mock::record(pin, true);
  return PassiveAdcStatus::Ok;
}

PassiveAdcStatus passiveAdcReadMv(uint8_t pin, uint16_t &millivolts) {
  if (mock::criticalDepth != 0) adcReadsInCritical++;
  mock::nowUs += 10;  // conversion time
  if (net.adcReadFails) return PassiveAdcStatus::ReadFailed;
  const int n = nodeOf(pin);
  if (n < 0 || (mock::pins[pin].mode & ANALOG) != ANALOG) return PassiveAdcStatus::ConfigFailed;
  double v[PROBE_NODE_COUNT];
  solveNodes(v);
  double volts = v[n];
  if (n == net.asymSenseNode && sinkEngaged()) volts = net.asymMv / 1000.0;
  long value = lround(volts * 1000.0);
  if (net.noiseMv != 0) {
    noiseFlip = !noiseFlip;
    value += noiseFlip ? static_cast<long>(net.noiseMv) : -static_cast<long>(net.noiseMv);
  }
  if (value < 0) value = 0;
  if (value > 3100) value = 3100;  // saturation near the top of the 12 dB range
  millivolts = static_cast<uint16_t>(value);
  return PassiveAdcStatus::Ok;
}

void passiveAdcEndSense(uint8_t pin) { pinMode(pin, INPUT); }

bool passiveAdcCalibrated() { return true; }

// ---- Simulated DHT11 -------------------------------------------------------

struct SimSensor {
  bool attached = false;
  ProbeMapping wiring = PROBE_PERMUTATIONS[0];
  uint8_t frame[5] = {60, 0, 25, 0, 85};
  bool respond = true;
  bool stuckLow = false;
  uint16_t ackLowUs = 80;
  uint16_t bitLowUs = 50;
  uint16_t oneHighUs = 70;
  uint8_t bitsSent = DHT_FRAME_BITS;  // fewer = truncated frame
  bool wrapMicrosAtRelease = false;

  bool powered = false;
  bool hostLow = false;
  bool responding = false;
  uint64_t poweredSinceUs = 0;
  uint64_t lowSinceUs = 0;
  uint64_t responseStartUs = 0;
  int responses = 0;
  int segCount = 0;
  uint16_t segUs[2 * DHT_FRAME_BITS + 4] = {};
  uint8_t segLevel[2 * DHT_FRAME_BITS + 4] = {};
};

static SimSensor sensor;
static Dht11Driver dht;          // production drivers, registered per test
static PassiveDriver passive;

static uint8_t wiringGpio(NodeRole role) { return gpioOf(probeNodeWithRole(sensor.wiring, role)); }

static void addSegment(uint16_t us, uint8_t level) {
  sensor.segUs[sensor.segCount] = us;
  sensor.segLevel[sensor.segCount] = level;
  sensor.segCount++;
}

static void buildResponse() {
  sensor.segCount = 0;
  addSegment(30, HIGH);              // sensor waits 20-40 us before answering
  addSegment(sensor.ackLowUs, LOW);  // acknowledgement
  addSegment(80, HIGH);
  for (int i = 0; i < sensor.bitsSent; i++) {
    addSegment(sensor.bitLowUs, LOW);
    addSegment(((sensor.frame[i / 8] >> (7 - i % 8)) & 1) ? sensor.oneHighUs : 27, HIGH);
  }
  if (sensor.bitsSent == DHT_FRAME_BITS) addSegment(50, LOW);  // end of frame
}

static int responseLevel(uint64_t t) {
  for (int i = 0; i < sensor.segCount; i++) {
    if (t < sensor.segUs[i]) return sensor.segLevel[i];
    t -= sensor.segUs[i];
  }
  return HIGH;  // released after the frame
}

// ---- Fake protected four-wire backend (replaces four_wire_backend.cpp) -----
// Stands in for a reviewed routing interface so the engine's four-wire
// path can be exercised. caps = 0 behaves like the shipped backend
// (nothing provided); a test must grant capabilities explicitly.

constexpr uint16_t kAllBackendCaps = kBackendFourWireBaseline | kBackendSupply5V;

struct FakeBackend {
  uint16_t caps = 0;
  bool refuse = false;     // the interface declines to route (e.g. its own fault)
  bool connected = false;
  int vcc = -1;            // routed nodes while connected
  int sig1 = -1;
  int sig2 = -1;
  int gnd = -1;
  uint64_t connectedAtUs = 0;
  int requests = 0;        // connect calls
  int connects = 0;        // accepted connections
  int violations = 0;      // connect while connected (no break), with a probe GPIO
                           // not plain INPUT or an interrupt attached, repeated or
                           // invalid nodes, or without the baseline capabilities
  int signalsDrivenAtOff = 0;  // supply removed while a probe GPIO was still an output
};
static FakeBackend backend;

static void sonarPowerLost();

uint16_t fourWireBackendCapabilities() { return backend.caps; }

bool fourWireBackendConnect(uint8_t vcc, uint8_t sig1, uint8_t sig2, uint8_t gnd) {
  backend.requests++;
  const uint8_t nodes[4] = {vcc, sig1, sig2, gnd};
  bool valid = true;
  for (int i = 0; i < 4; i++) {
    if (nodes[i] >= PROBE_NODE_COUNT) valid = false;
    for (int j = 0; j < i; j++) {
      if (nodes[i] == nodes[j]) valid = false;
    }
  }
  if (backend.connected || !allProbesPlainInput() || anyProbeIsrAttached() || !valid ||
      (backend.caps & kBackendFourWireBaseline) != kBackendFourWireBaseline) {
    backend.violations++;
  }
  if (backend.refuse) return false;
  backend.connected = true;
  backend.vcc = vcc;
  backend.sig1 = sig1;
  backend.sig2 = sig2;
  backend.gnd = gnd;
  backend.connects++;
  backend.connectedAtUs = mock::nowUs;
  return true;
}

void fourWireBackendDisconnect() {
  if (!backend.connected) return;
  for (uint8_t g : PROBE_NODE_GPIO) {
    if (mock::isOutput(g)) {
      backend.signalsDrivenAtOff++;
      break;
    }
  }
  backend.connected = false;
  backend.vcc = backend.sig1 = backend.sig2 = backend.gnd = -1;
  sonarPowerLost();
}

// ---- Simulated HC-SR04 -------------------------------------------------------
// Powered only through the fake backend with its own VCC and GND routed. A
// TRIG pulse of >= 10 us released while powered schedules one echo on its
// ECHO node; advanceTo() delivers the edges to the attached ISR at their
// own instants, so the driver may process them much later.

struct SimSonar {
  bool attached = false;
  int wiring = 0;               // PROBE_FOUR_WIRE_MAPPINGS index (VCC TRIG ECHO GND)
  uint32_t riseDelayUs = 460;   // trigger release to ECHO rising
  uint32_t widthUs = 7157;      // ECHO HIGH time: 123.4 cm
  bool respond = true;          // false: ECHO never rises (no echo, or stuck LOW)
  bool stuckHigh = false;       // ECHO reads HIGH regardless
  bool coupledToTrig = false;   // ECHO follows TRIG (a wiring/crosstalk fault)
  uint8_t level = LOW;
  bool trigHigh = false;
  uint64_t trigRiseUs = 0;
  uint64_t riseAtUs = 0;        // scheduled edges, 0 = none
  uint64_t fallAtUs = 0;
  int pulses = 0;               // TRIG HIGH pulses seen, answered or not
  int triggers = 0;             // valid triggers answered: fresh measurements
  uint64_t triggerUs[128] = {};
  uint64_t minPulseUs = UINT64_MAX;
};
static SimSonar sonar;
static UltrasonicDriver ultrasonic;  // production driver, registered per test

static uint8_t sonarGpio(NodeRole role) {
  return gpioOf(probeNodeWithRole(PROBE_FOUR_WIRE_MAPPINGS[sonar.wiring], role));
}

static bool sonarPowered() {
  const ProbeMapping &m = PROBE_FOUR_WIRE_MAPPINGS[sonar.wiring];
  return sonar.attached && backend.connected && backend.vcc == probeNodeWithRole(m, NodeRole::Vcc) &&
         backend.gnd == probeNodeWithRole(m, NodeRole::Gnd);
}

// Moves ECHO and reports the edge to whatever ISR is attached to it.
static void setEcho(uint8_t level) {
  if (sonar.level == level) return;
  sonar.level = level;
  mock::fireInterrupt(sonarGpio(NodeRole::Sig2));
}

static void sonarPowerLost() {
  sonar.riseAtUs = sonar.fallAtUs = 0;
  if (sonar.attached) setEcho(LOW);
}

static void sonarOnTrig() {
  const bool high = mock::drivesHigh(sonarGpio(NodeRole::Sig1));
  if (high == sonar.trigHigh) return;
  sonar.trigHigh = high;
  if (sonar.coupledToTrig) setEcho(high ? HIGH : LOW);
  if (high) {
    sonar.trigRiseUs = mock::nowUs;
    sonar.pulses++;
    return;
  }
  const uint64_t width = mock::nowUs - sonar.trigRiseUs;
  if (width < sonar.minPulseUs) sonar.minPulseUs = width;
  if (!sonarPowered() || !sonar.respond || width < 10 || sonar.riseAtUs != 0 || sonar.fallAtUs != 0) {
    return;
  }
  if (sonar.triggers < 128) sonar.triggerUs[sonar.triggers] = mock::nowUs;
  sonar.triggers++;
  sonar.riseAtUs = mock::nowUs + sonar.riseDelayUs;
  sonar.fallAtUs = sonar.riseAtUs + sonar.widthUs;
}

static void advanceTo(uint64_t untilUs) {
  for (;;) {
    const uint64_t next = sonar.riseAtUs != 0 ? sonar.riseAtUs : sonar.fallAtUs;
    if (next == 0 || next > untilUs) break;
    if (next > mock::nowUs) mock::nowUs = next;
    if (sonar.riseAtUs != 0) {
      sonar.riseAtUs = 0;
      setEcho(HIGH);
    } else {
      sonar.fallAtUs = 0;
      setEcho(LOW);
    }
  }
  if (untilUs > mock::nowUs) mock::nowUs = untilUs;
}

// Called after every pinMode()/digitalWrite().
static void onPinChange(uint8_t pin, bool isPinMode) {
  if (isPinMode && mock::criticalDepth != 0) lockedPinModes++;
  if (nodeOf(pin) >= 0 && mock::drivesHigh(pin)) highDriveEvents++;
  if (sonar.attached && pin == sonarGpio(NodeRole::Sig1)) sonarOnTrig();
  if (!sensor.attached) return;

  const bool poweredNow = mock::drivesHigh(wiringGpio(NodeRole::Vcc)) &&
                          mock::drivesLow(wiringGpio(NodeRole::Gnd));
  if (poweredNow && !sensor.powered) sensor.poweredSinceUs = mock::nowUs;
  if (!poweredNow) sensor.responding = false;
  sensor.powered = poweredNow;

  const bool lowNow = mock::drivesLow(wiringGpio(NodeRole::Data));
  if (lowNow && !sensor.hostLow) {
    sensor.lowSinceUs = mock::nowUs;
    sensor.responding = false;
  }
  if (!lowNow && sensor.hostLow && sensor.powered && sensor.respond &&
      mock::nowUs - sensor.poweredSinceUs >= 1000000 && mock::nowUs - sensor.lowSinceUs >= 18000) {
    buildResponse();
    sensor.responding = true;
    sensor.responseStartUs = mock::nowUs;
    sensor.responses++;
    if (sensor.wrapMicrosAtRelease) {  // micros() wraps ~40 us into this response
      mock::microsOffset = static_cast<uint32_t>(0xFFFFFFFFu - 40u - static_cast<uint32_t>(mock::nowUs));
    }
  }
  sensor.hostLow = lowNow;
}

static int readHook(uint8_t pin) {
  if (sonar.attached && pin == sonarGpio(NodeRole::Sig2)) return sonar.stuckHigh ? HIGH : sonar.level;
  if (sensor.attached && pin == wiringGpio(NodeRole::Data)) {
    if (sensor.stuckLow) return LOW;
    if (sensor.responding) return responseLevel(mock::nowUs - sensor.responseStartUs);
  }
  if (flakyA && pin == gpioOf(0) && mock::pins[pin].mode == INPUT_PULLUP) {
    return (flakyReads++ % 2 == 0) ? HIGH : LOW;
  }
  const int n = nodeOf(pin);
  if (n < 0) return -1;
  double v[PROBE_NODE_COUNT];
  solveNodes(v);
  return v[n] >= 1.5 ? HIGH : LOW;  // simulated input threshold
}

// Call useDrivers() first: init resets every registered driver. backendCaps
// is what the fake four-wire backend provides (0 = like the shipped one).
static void resetWorld(uint64_t startUs = 1000000ULL, uint16_t backendCaps = 0) {
  mock::reset(startUs);
  sensor = SimSensor();
  sonar = SimSonar();
  backend = FakeBackend();
  backend.caps = backendCaps;
  netReset();
  flakyA = false;
  flakyReads = 0;
  senseCount = 0;
  highDriveEvents = 0;
  attemptCount = 0;
  mock::readHook = readHook;
  mock::changeHook = onPinChange;
  initProbeEngine();
}

[[maybe_unused]] static void powerByHand(const ProbeMapping &m) {  // the engine's job, done manually
  resetTestPins();
  const uint8_t vcc = gpioOf(probeNodeWithRole(m, NodeRole::Vcc));
  const uint8_t gnd = gpioOf(probeNodeWithRole(m, NodeRole::Gnd));
  digitalWrite(gnd, LOW);
  pinMode(gnd, OUTPUT);
  digitalWrite(vcc, HIGH);
  pinMode(vcc, OUTPUT);
  pinMode(gpioOf(probeNodeWithRole(m, NodeRole::Data)), INPUT_PULLUP);
  mock::nowUs += DHT_POWER_STABILIZE_MS * 1000ULL;
}

// ---- Fake driver: proves the engine is driver-agnostic ---------------------

class FakeDriver final : public SensorDriver {
 public:
  FakeDriver(int id, const char *name, ProbeTopology topology = ProbeTopology::ThreeWirePowered,
             ExcitationPolicy excitation = ExcitationPolicy::ProtectedActive)
      : req{topology, excitation, 300, 500, 700, 1000, 0, {nullptr, nullptr}}, id_(id), name_(name) {}

  DriverRequirements req;
  int pendingCalls = 0;  // Pending results before each operation completes
  int acceptSlot = -1;   // probe() succeeds only on this mapping/pair index
  bool samplesSucceed = true;

  int probeStarts = 0;
  int sampleStarts = 0;
  int calls = 0;
  int resets = 0;
  int unsafeResets = 0;     // reset() while a probe pin was not plain INPUT
  int unpoweredStarts = 0;  // powered operation started without exactly its mapping powered
                            // (three-wire: VCC/GND GPIOs; four-wire: backend routes, GPIOs INPUT)
  int energisedStarts = 0;  // two-terminal operation started with a probe pin not plain INPUT

  const char *getName() const override { return name_; }
  DriverOperationStatus getOperationStatus() const override { return status_; }
  const DriverRequirements &getRequirements() const override { return req; }
  const char *getFailureReason() const override {
    return status_ == DriverOperationStatus::Failed ? "fake-failed" : "";
  }

  bool probe(const ProbeContext &context) override {
    calls++;
    if (context.topology != req.topology) {
      status_ = DriverOperationStatus::Failed;
      return false;
    }
    if (status_ != DriverOperationStatus::Pending) {
      probeStarts++;
      context_ = context;
      slot_ = slotOf(context);
      bound_ = false;
      start(false);
    }
    if (remaining_ > 0) {
      remaining_--;
      return false;
    }
    bound_ = slot_ == acceptSlot;
    status_ = bound_ ? DriverOperationStatus::Succeeded : DriverOperationStatus::Failed;
    return bound_;
  }

  bool readSample(char *line1, char *line2, size_t maxLen) override {
    calls++;
    if (line1 == nullptr || line2 == nullptr || maxLen == 0) {
      status_ = DriverOperationStatus::Failed;
      return false;
    }
    line1[0] = line2[0] = '\0';
    if (status_ != DriverOperationStatus::Pending) {
      if (!bound_) {
        status_ = DriverOperationStatus::Failed;
        return false;
      }
      sampleStarts++;
      start(true);
    }
    if (remaining_ > 0) {
      remaining_--;
      return false;
    }
    status_ = samplesSucceed ? DriverOperationStatus::Succeeded : DriverOperationStatus::Failed;
    if (!samplesSucceed) return false;
    snprintf(line1, maxLen, "%s #%d", name_, sampleStarts);
    snprintf(line2, maxLen, "second line");
    return true;
  }

  void reset() override {
    resets++;
    if (!allProbesPlainInput()) unsafeResets++;
    status_ = DriverOperationStatus::Idle;
    bound_ = false;
    remaining_ = 0;
  }

 private:
  void start(bool sample) {
    status_ = DriverOperationStatus::Pending;
    remaining_ = pendingCalls;
    if (req.topology == ProbeTopology::ThreeWirePowered) {
      if (!(mock::drivesHigh(context_.powered.vcc) && mock::drivesLow(context_.powered.gnd))) {
        unpoweredStarts++;
      }
    } else if (req.topology == ProbeTopology::FourWirePowered) {
      const FourWirePins &p = context_.fourWire;
      const bool routed = backend.connected && gpioOf(backend.vcc) == p.vcc &&
                          gpioOf(backend.sig1) == p.sig1 && gpioOf(backend.sig2) == p.sig2 &&
                          gpioOf(backend.gnd) == p.gnd;
      if (!routed || !allProbesPlainInput()) unpoweredStarts++;
    } else if (!allProbesPlainInput()) {
      energisedStarts++;
    }
    if (attemptCount < 256) attempts[attemptCount++] = Attempt{id_, slot_, sample, req.topology, mock::nowUs};
  }

  int id_;
  const char *name_;
  DriverOperationStatus status_ = DriverOperationStatus::Idle;
  ProbeContext context_{};
  bool bound_ = false;
  int remaining_ = 0;
  int slot_ = -1;
};

// ---- Event-log replay: one Session per driven interval ---------------------

struct Session {
  uint64_t startUs = 0;
  uint64_t endUs = UINT64_MAX;  // UINT64_MAX = still driven
  int vcc = -1;   // -1 for passive (open-drain sink only) intervals
  int gnd = -1;
  int data = -1;  // known for drivers that pull a line low (DHT start, passive sink)
  int starts = 0;
  uint64_t firstStartUs = 0;
  uint8_t usedMask = 0;  // nodes that left plain INPUT at any time during the session
};

struct SessionLog {
  Session s[96];
  int count = 0;
  int violations = 0;
  uint64_t startTimes[256] = {};
  int startCount = 0;
};

// Replays every probe pin change and flags sequencing violations: a
// push-pull drive while precheck pulls remain, two supplies, VCC without GND
// driven low (covers GND-first power-up and GND-last teardown), DATA (or
// any node but GND) not released before VCC, or DATA driven high push-pull
// (shows as two supplies). For GPIO-powered (three-wire) and passive
// activity only: a four-wire TRIG pulse would read as a supply here.
[[maybe_unused]] static SessionLog collectSessions() {
  SessionLog log;
  uint8_t mode[PROBE_NODE_COUNT] = {};
  uint8_t latch[PROBE_NODE_COUNT] = {};
  auto out = [&](int n) { return (mode[n] & OUTPUT) == OUTPUT; };
  auto high = [&](int n) { return out(n) && !(mode[n] & OPEN_DRAIN) && latch[n] == HIGH; };
  auto lowPP = [&](int n) { return out(n) && !(mode[n] & OPEN_DRAIN) && latch[n] == LOW; };
  auto lowOD = [&](int n) { return out(n) && (mode[n] & OPEN_DRAIN) && latch[n] == LOW; };

  bool active = false;
  Session cur;
  CHECK(!mock::eventOverflow);
  for (size_t i = 0; i < mock::eventCount; i++) {
    const mock::Event &e = mock::events[i];
    const int n = nodeOf(e.pin);
    if (n < 0) continue;

    int highBefore = -1;
    for (int k = 0; k < PROBE_NODE_COUNT; k++) {
      if (high(k)) highBefore = k;
    }
    const bool odBefore = lowOD(n);
    mode[n] = e.mode;
    latch[n] = e.latch;

    int highs = 0, highNode = -1, ppLows = 0, ppLowNode = -1, odNode = -1;
    for (int k = 0; k < PROBE_NODE_COUNT; k++) {
      if (high(k)) { highs++; highNode = k; }
      if (lowPP(k)) { ppLows++; ppLowNode = k; }
      if (lowOD(k)) odNode = k;
    }
    const bool driven = highs > 0 || ppLows > 0 || odNode >= 0;

    if (driven && !active) {
      active = true;
      cur = Session();
      cur.startUs = e.us;
      if (highs > 0 || ppLows > 0) {  // powered: precheck pulls must be gone
        for (int k = 0; k < PROBE_NODE_COUNT; k++) {
          if (k != n && mode[k] != INPUT) log.violations++;
        }
      }
    }
    if (highs > 1) log.violations++;
    if (highNode >= 0 && ppLows == 0) log.violations++;
    if (highBefore >= 0 && highNode < 0 && cur.gnd >= 0) {  // VCC just released
      for (int k = 0; k < PROBE_NODE_COUNT; k++) {
        if (k != cur.gnd && mode[k] != INPUT) log.violations++;  // DATA was not released first
      }
    }
    if (active) {
      for (int k = 0; k < PROBE_NODE_COUNT; k++) {
        if (mode[k] != INPUT) cur.usedMask |= static_cast<uint8_t>(1u << k);
      }
      if (highNode >= 0) cur.vcc = highNode;
      if (ppLowNode >= 0) cur.gnd = ppLowNode;
      if (odNode >= 0) cur.data = odNode;
      if (odNode == n && !odBefore) {  // a new open-drain LOW (DHT start or passive sink)
        if (cur.starts++ == 0) cur.firstStartUs = e.us;
        if (log.startCount < 256) log.startTimes[log.startCount++] = e.us;
      }
    }
    if (!driven && active) {
      active = false;
      cur.endUs = e.us;
      if (log.count < 96) log.s[log.count++] = cur;
    }
  }
  if (active && log.count < 96) log.s[log.count++] = cur;
  return log;
}

[[maybe_unused]] static bool sessionUses(const Session &s, const ProbeMapping &m) {
  return s.vcc == probeNodeWithRole(m, NodeRole::Vcc) && s.gnd == probeNodeWithRole(m, NodeRole::Gnd) &&
         s.data == probeNodeWithRole(m, NodeRole::Data);
}

[[maybe_unused]] static bool sessionPowers(const Session &s, const ProbeMapping &m) {
  return s.vcc == probeNodeWithRole(m, NodeRole::Vcc) && s.gnd == probeNodeWithRole(m, NodeRole::Gnd);
}

// Every node the mapping leaves unused stayed plain INPUT throughout.
[[maybe_unused]] static bool unusedStayedInput(const Session &s, const ProbeMapping &m) {
  for (int n = 0; n < PROBE_NODE_COUNT; n++) {
    if (m.role[n] == NodeRole::Unused && (s.usedMask & (1u << n)) != 0) return false;
  }
  return true;
}

[[maybe_unused]] static int poweredSessions(const SessionLog &log) {
  int count = 0;
  for (int i = 0; i < log.count; i++) {
    if (log.s[i].vcc >= 0) count++;
  }
  return count;
}

// ---- Driver contracts, driven directly (every build) -----------------------

static void testDht11DriverContract() {
  useDrivers();  // the engine stays idle; this test drives the driver itself
  resetWorld();
  Dht11Driver d;
  char l1[PROBE_TELEMETRY_TEXT_LEN];
  char l2[PROBE_TELEMETRY_TEXT_LEN];
  const ProbeContext ctx = poweredContext(0);
  const uint8_t vcc = ctx.powered.vcc, data = ctx.powered.data, gnd = ctx.powered.gnd;
  CHECK(same(d.getName(), "DHT11 Temp & Hum"));
  CHECK(d.getRequirements().topology == ProbeTopology::ThreeWirePowered);
  CHECK(d.getRequirements().excitation == ExcitationPolicy::ProtectedActive);
  CHECK(d.getRequirements().powerStabilizeMs == DHT_POWER_STABILIZE_MS);
  CHECK(d.getRequirements().minAcquisitionIntervalMs == DHT_MIN_START_INTERVAL_MS);
  CHECK(d.getRequirements().sampleIntervalMs == DHT_READ_INTERVAL_MS);

  // Invalid buffers are rejected without being dereferenced.
  CHECK(!d.readSample(nullptr, l2, sizeof l2));
  CHECK(d.getOperationStatus() == DriverOperationStatus::Failed);
  CHECK(!d.readSample(l1, nullptr, sizeof l1));
  memset(l1, 'x', sizeof l1);
  CHECK(!d.readSample(l1, l2, 0));
  CHECK(l1[0] == 'x');  // zero capacity: untouched

  // Sampling before a successful probe fails cleanly with empty outputs.
  d.reset();
  CHECK(d.getOperationStatus() == DriverOperationStatus::Idle);
  CHECK(!d.readSample(l1, l2, sizeof l1));
  CHECK(d.getOperationStatus() == DriverOperationStatus::Failed);
  CHECK(l1[0] == '\0' && l2[0] == '\0' && same(d.getFailureReason(), "not-probed"));

  // A two-terminal context is refused without touching any pin.
  size_t quiet = mock::eventCount;
  CHECK(!d.probe(pairContext(0)));
  CHECK(same(d.getFailureReason(), "topology-mismatch") && mock::eventCount == quiet);

  // Probe: Pending for the 20 ms start pulse, then one genuine frame. The
  // driver touches only DATA.
  sensor.attached = true;
  sensor.wiring = PROBE_PERMUTATIONS[0];
  powerByHand(PROBE_PERMUTATIONS[0]);
  d.reset();
  const size_t eventsBefore = mock::eventCount;
  int pending = 0;
  while (!d.probe(ctx) && pending < 100) {
    CHECK(d.getOperationStatus() == DriverOperationStatus::Pending);
    pending++;
    mock::nowUs += 1000;
  }
  CHECK(d.getOperationStatus() == DriverOperationStatus::Succeeded);
  CHECK(pending == static_cast<int>(DHT_START_LOW_MS));
  CHECK(sensor.responses == 1);
  for (size_t i = eventsBefore; i < mock::eventCount; i++) CHECK(mock::events[i].pin == data);
  CHECK(mock::drivesHigh(vcc) && mock::drivesLow(gnd));
  CHECK(mock::criticalDepth == 0);

  // A sample is a new acquisition; a small buffer is truncated but
  // terminated, and stays empty while the operation is pending.
  mock::nowUs += DHT_READ_INTERVAL_MS * 1000ULL;
  char s1[6];
  char s2[6];
  memset(s1, 'x', sizeof s1);
  memset(s2, 'x', sizeof s2);
  CHECK(!d.readSample(s1, s2, sizeof s1));
  CHECK(d.getOperationStatus() == DriverOperationStatus::Pending);
  CHECK(s1[0] == '\0' && s2[0] == '\0');
  mock::nowUs += DHT_START_LOW_MS * 1000ULL;
  CHECK(d.readSample(s1, s2, sizeof s1));
  CHECK(strcmp(s1, "T: 25") == 0 && s2[0] == '\0');
  CHECK(sensor.responses == 2);

  // Completed samples are never re-reported: the next call starts anew.
  mock::nowUs += DHT_READ_INTERVAL_MS * 1000ULL;
  CHECK(!d.readSample(l1, l2, sizeof l1));
  CHECK(d.getOperationStatus() == DriverOperationStatus::Pending);
  CHECK(sensor.responses == 2);

  // Overlap is refused: probe() during a pending sample fails and releases
  // DATA to plain INPUT (never driven, never pulled up).
  CHECK(!d.probe(ctx));
  CHECK(d.getOperationStatus() == DriverOperationStatus::Failed);
  CHECK(same(d.getFailureReason(), "contract-violation"));
  CHECK(mock::pins[data].mode == INPUT);

  // A failed sample clears the outputs.
  powerByHand(PROBE_PERMUTATIONS[0]);
  d.reset();
  while (!d.probe(ctx) && d.getOperationStatus() == DriverOperationStatus::Pending) {
    mock::nowUs += 1000;
  }
  CHECK(d.getOperationStatus() == DriverOperationStatus::Succeeded);
  sensor.respond = false;
  mock::nowUs += DHT_READ_INTERVAL_MS * 1000ULL;
  memset(l1, 'x', sizeof l1);
  while (!d.readSample(l1, l2, sizeof l1) &&
         d.getOperationStatus() == DriverOperationStatus::Pending) {
    mock::nowUs += 1000;
  }
  CHECK(d.getOperationStatus() == DriverOperationStatus::Failed);
  CHECK(l1[0] == '\0' && same(d.getFailureReason(), dhtErrorName(DhtError::NoResponse)));
  sensor.respond = true;

  // Changing arguments mid-operation is refused.
  powerByHand(PROBE_PERMUTATIONS[0]);
  d.reset();
  CHECK(!d.probe(ctx));
  ProbeContext swapped = ctx;
  swapped.powered.vcc = ctx.powered.gnd;
  swapped.powered.gnd = ctx.powered.vcc;
  CHECK(!d.probe(swapped));
  CHECK(d.getOperationStatus() == DriverOperationStatus::Failed);

  // reset() during a start pulse only releases DATA to plain INPUT; after the
  // engine has made the pins safe, reset() (twice) touches nothing.
  powerByHand(PROBE_PERMUTATIONS[0]);
  d.reset();
  CHECK(!d.probe(ctx));
  CHECK(mock::drivesLow(data));
  d.reset();
  CHECK(mock::pins[data].mode == INPUT);
  CHECK(d.getOperationStatus() == DriverOperationStatus::Idle);
  resetTestPins();
  quiet = mock::eventCount;
  d.reset();
  d.reset();
  CHECK(mock::eventCount == quiet);
  CHECK(mock::criticalDepth == 0);
}

static void testPassiveDriverContract() {
  useDrivers();
  resetWorld();
  PassiveDriver d;
  char l1[PROBE_TELEMETRY_TEXT_LEN];
  char l2[PROBE_TELEMETRY_TEXT_LEN];
  CHECK(d.getRequirements().topology == ProbeTopology::TwoTerminal);
  CHECK(d.getRequirements().excitation == ExcitationPolicy::WeakPullOnly);
  CHECK(same(d.getName(), "Passive Component"));

  // Every pair (all six, D included), both orientations (sense first
  // terminal, then second); the two unused nodes are never touched; the
  // sink is open-drain only.
  for (int i = 0; i < PROBE_PAIR_COUNT; i++) {
    const ProbeMapping &m = PROBE_PAIRS[i];
    const int t0 = probeNodeWithRole(m, NodeRole::Terminal, 0);
    const int t1 = probeNodeWithRole(m, NodeRole::Terminal, 1);
    const int unused = probeNodeWithRole(m, NodeRole::Unused, 0);
    const int unused2 = probeNodeWithRole(m, NodeRole::Unused, 1);
    netReset();
    connect(t0, t1, 10000);
    resetTestPins();
    d.reset();
    senseCount = 0;
    highDriveEvents = 0;
    const size_t from = mock::eventCount;
    CHECK(driveProbe(d, pairContext(i)));
    CHECK(same(d.getName(), "Resistive Component"));
    CHECK(senseCount == 2 && senseLog[0] == gpioOf(t0) && senseLog[1] == gpioOf(t1));
    int sinks = 0;
    uint8_t sinkPins[4] = {0, 0, 0, 0};
    bool unusedTouched = false;
    bool pushPull = false;
    for (size_t e = from; e < mock::eventCount; e++) {
      const mock::Event &ev = mock::events[e];
      if (ev.pin == gpioOf(unused) || ev.pin == gpioOf(unused2)) unusedTouched = true;
      if (ev.mode == OUTPUT) pushPull = true;
      if (ev.isPinMode && ev.mode == OUTPUT_OPEN_DRAIN && sinks < 4) sinkPins[sinks++] = ev.pin;
    }
    CHECK(!unusedTouched && !pushPull);
    CHECK(sinks == 2 && sinkPins[0] == gpioOf(t1) && sinkPins[1] == gpioOf(t0));
    CHECK(highDriveEvents == 0 && allProbesPlainInput());
    mock::nowUs += PASSIVE_SAMPLE_PERIOD_MS * 1000ULL;
    CHECK(driveSample(d, l1, l2, sizeof l1));
    CHECK(same(l1, "R~ 10.0 kOhm") && same(l2, "Signal: 81%"));
    CHECK(allProbesPlainInput());
  }

  // A contact gives a valid 0 mV reading, distinct from any ADC failure.
  netReset();
  connect(0, 1, 1.0);
  resetTestPins();
  d.reset();
  CHECK(driveProbe(d, pairContext(0)));
  CHECK(same(d.getName(), "Contact / Low R"));
  mock::nowUs += PASSIVE_SAMPLE_PERIOD_MS * 1000ULL;
  CHECK(driveSample(d, l1, l2, sizeof l1));
  CHECK(same(l1, "State: CLOSED") && same(l2, "R~ < 2.1 kOhm"));

  struct Case {
    void (*setup)();
    const char *reason;
    bool measurementError;
  };
  const Case cases[] = {
      {[] {}, "open-circuit", false},                                // empty pair
      {[] { connect(0, 1, 1e6); }, "open-circuit", false},           // saturated: a bound, not a value
      {[] { connect(0, 1, 1.0); net.adcReadFails = true; }, "adc-read-failed", true},
      {[] { net.adcBeginFails = true; }, "adc-config-failed", true},  // unsupported ADC setup
      {[] { net.adcUnavailable = true; }, "adc-unavailable", true},
      {[] { connect(0, 1, 10000); net.pullupIneffective = true; }, "pullup-unverified", true},
      {[] { net.asymSenseNode = 0; net.asymMv = 600; }, "asymmetric", false},  // junction-like
      {[] { connect(0, 1, 10000); net.noiseMv = 60; }, "unstable", false},
  };
  for (const Case &c : cases) {
    netReset();
    c.setup();
    resetTestPins();
    d.reset();
    const size_t from = mock::eventCount;
    CHECK(!driveProbe(d, pairContext(0)));
    CHECK(d.getOperationStatus() == DriverOperationStatus::Failed);
    CHECK(same(d.getFailureReason(), c.reason));
    CHECK(d.failureIsMeasurementError() == c.measurementError);
    CHECK(allProbesPlainInput());
    if (c.measurementError) {  // the sink is never engaged on an unverified measurement
      for (size_t e = from; e < mock::eventCount; e++) {
        CHECK(!(mock::events[e].isPinMode && mock::events[e].mode == OUTPUT_OPEN_DRAIN));
      }
    }
  }

  // Wrong topology: refused without touching any pin.
  netReset();
  resetTestPins();
  d.reset();
  size_t quiet = mock::eventCount;
  CHECK(!d.probe(poweredContext(0)));
  CHECK(same(d.getFailureReason(), "topology-mismatch") && mock::eventCount == quiet);

  // Buffers: rejected without dereferencing; small buffers stay terminated.
  connect(0, 1, 10000);
  d.reset();
  CHECK(!d.readSample(nullptr, l2, sizeof l2) && same(d.getFailureReason(), "invalid-buffer"));
  memset(l1, 'x', sizeof l1);
  CHECK(!d.readSample(l1, l2, 0) && l1[0] == 'x');
  CHECK(!d.readSample(l1, l2, sizeof l1) && same(d.getFailureReason(), "not-probed"));
  CHECK(l1[0] == '\0' && l2[0] == '\0');
  CHECK(driveProbe(d, pairContext(0)));
  mock::nowUs += PASSIVE_SAMPLE_PERIOD_MS * 1000ULL;
  char s1[6];
  char s2[6];
  CHECK(driveSample(d, s1, s2, sizeof s1));
  CHECK(same(s1, "R~ 10") && same(s2, "Signa"));

  // reset() cancels a pending measurement; nothing re-excites the pair
  // afterwards, and reset() after that touches nothing.
  resetTestPins();
  d.reset();
  CHECK(!d.probe(pairContext(0)));
  CHECK(d.getOperationStatus() == DriverOperationStatus::Pending);
  CHECK((mock::pins[gpioOf(0)].mode & ANALOG) == ANALOG);
  d.reset();
  CHECK(allProbesPlainInput() && d.getOperationStatus() == DriverOperationStatus::Idle);
  quiet = mock::eventCount;
  mock::nowUs += 500000;
  CHECK(!d.readSample(l1, l2, sizeof l1) && same(d.getFailureReason(), "not-probed"));
  CHECK(mock::eventCount == quiet);

  CHECK(!d.probe(pairContext(0)));
  for (int guard = 0; !sinkEngaged() && guard < 100; guard++) {
    mock::nowUs += 1000;
    d.probe(pairContext(0));
  }
  CHECK(sinkEngaged());
  d.reset();
  CHECK(allProbesPlainInput() && !sinkEngaged());
  quiet = mock::eventCount;
  d.reset();
  d.reset();
  CHECK(mock::eventCount == quiet);
  CHECK(highDriveEvents == 0);
}

// Passive measurement, then a DHT probe on the same pins: the passive
// teardown leaves plain digital inputs that the DHT protocol can use.
static void testPassiveToDhtHandoff() {
  useDrivers();
  resetWorld();
  PassiveDriver pd;
  Dht11Driver dd;
  connect(0, 1, 10000);
  CHECK(driveProbe(pd, pairContext(0)));
  pd.reset();
  for (uint8_t g : PROBE_NODE_GPIO) CHECK(mock::pins[g].mode == INPUT);  // no analog/pull state left
  netReset();
  sensor.attached = true;
  sensor.wiring = PROBE_PERMUTATIONS[0];
  powerByHand(PROBE_PERMUTATIONS[0]);
  CHECK(driveProbe(dd, poweredContext(0)));
  CHECK(sensor.responses == 1);
  resetTestPins();
}

// ---- Ultrasonic driver, driven directly (every build) ----------------------

// Routes the simulated sonar's own mapping through the fake backend, as the
// engine would before an attempt (the engine's job, done manually).
static void powerSonarByHand() {
  resetTestPins();
  const ProbeMapping &m = PROBE_FOUR_WIRE_MAPPINGS[sonar.wiring];
  CHECK(fourWireBackendConnect(static_cast<uint8_t>(probeNodeWithRole(m, NodeRole::Vcc)),
                               static_cast<uint8_t>(probeNodeWithRole(m, NodeRole::Sig1)),
                               static_cast<uint8_t>(probeNodeWithRole(m, NodeRole::Sig2)),
                               static_cast<uint8_t>(probeNodeWithRole(m, NodeRole::Gnd))));
}

static void testUltrasonicEchoMath() {
  uint32_t echo = 0;
  float cm = 0.0f;
  CHECK(ultrasonicCheckEcho(1000, 1460, 1460 + 7157, echo, cm) == nullptr);
  CHECK(echo == 7157 && cm == 7157 / 58.0f);  // distanceCm = echoDurationUs / 58.0f
  CHECK(static_cast<long>(cm * 10.0f + 0.5f) == 1234);

  // Range 2.0-300.0 cm inclusive: 116 us and 17400 us.
  CHECK(ultrasonicCheckEcho(0, 500, 500 + 116, echo, cm) == nullptr && cm == 2.0f);
  CHECK(ultrasonicCheckEcho(0, 500, 500 + 17400, echo, cm) == nullptr && cm == 300.0f);
  echo = 1;
  cm = 1.0f;
  CHECK(same(ultrasonicCheckEcho(0, 500, 500 + 115, echo, cm), "out-of-range"));
  CHECK(same(ultrasonicCheckEcho(0, 500, 500 + 17401, echo, cm), "out-of-range"));
  CHECK(same(ultrasonicCheckEcho(0, 500, 500, echo, cm), "out-of-range"));  // zero width
  CHECK(echo == 1 && cm == 1.0f);  // a rejected echo leaves the outputs alone

  // One 25 ms budget from the trigger reference to the falling edge.
  CHECK(ultrasonicCheckEcho(0, 20000, 25000, echo, cm) == nullptr && echo == 5000);
  CHECK(same(ultrasonicCheckEcho(0, 20000, 25001, echo, cm), "echo-timeout"));
  CHECK(same(ultrasonicCheckEcho(0, 24990, 25100, echo, cm), "echo-timeout"));
  // Edge order and activity before the trigger reference.
  CHECK(same(ultrasonicCheckEcho(1000, 990, 2000, echo, cm), "pre-trigger-echo"));
  CHECK(same(ultrasonicCheckEcho(1000, 1500, 1400, echo, cm), "malformed-echo"));

  // micros() rollover: before the rise, and between the two edges.
  const uint32_t early = 0xFFFFFF00u;
  CHECK(ultrasonicCheckEcho(early, early + 460u, early + 460u + 7157u, echo, cm) == nullptr && echo == 7157);
  const uint32_t mid = 0xFFFFF000u;
  CHECK(ultrasonicCheckEcho(mid, mid + 460u, mid + 460u + 7157u, echo, cm) == nullptr && echo == 7157);
  CHECK(same(ultrasonicCheckEcho(early, early + 20000u, early + 25001u, echo, cm), "echo-timeout"));
  CHECK(same(ultrasonicCheckEcho(early, early - 5u, early + 7000u, echo, cm), "pre-trigger-echo"));
}

static int faultPulses = 0;       // TRIG pulses sent by the sample in sampleFailure()
static uint32_t faultMs = 0;      // how long that sample took to finish

// Probes the simulated sonar with a fresh driver, applies `fault`, then
// runs one sample; `afterTrigger` runs right after the sample's first call.
// Returns why the sample failed (nullptr if it succeeded) and checks that a
// failure leaves no drive, no capture and empty outputs behind.
static const char *sampleFailure(void (*fault)(), void (*afterTrigger)() = nullptr) {
  useDrivers();
  resetWorld(1000000ULL, kAllBackendCaps);
  sonar.attached = true;
  powerSonarByHand();
  UltrasonicDriver d;
  CHECK(driveProbe(d, fourWireContext(0)));
  advanceTo(mock::nowUs + 100000);
  fault();
  const int pulsesBefore = sonar.pulses;
  const uint64_t startUs = mock::nowUs;
  char l1[PROBE_TELEMETRY_TEXT_LEN];
  char l2[PROBE_TELEMETRY_TEXT_LEN];
  bool ok = d.readSample(l1, l2, sizeof l1);
  if (afterTrigger != nullptr) afterTrigger();
  if (!ok && d.getOperationStatus() == DriverOperationStatus::Pending) {
    advanceTo(mock::nowUs + 1000);
    ok = driveSample(d, l1, l2, sizeof l1);
  }
  faultPulses = sonar.pulses - pulsesBefore;
  faultMs = static_cast<uint32_t>((mock::nowUs - startUs) / 1000);
  const char *reason = ok ? nullptr : d.getFailureReason();
  if (!ok) {
    CHECK(d.getOperationStatus() == DriverOperationStatus::Failed);
    CHECK(l1[0] == '\0' && l2[0] == '\0');
    CHECK(!anyProbeIsrAttached());                                    // capture released at once
    CHECK(mock::pins[fourWireContext(0).fourWire.sig1].mode == INPUT);  // TRIG no longer driven
  }
  d.reset();
  return reason;
}

static void testUltrasonicDriverContract() {
  useDrivers();  // the engine stays idle; this test drives the driver itself
  resetWorld(1000000ULL, kAllBackendCaps);
  UltrasonicDriver d;
  char l1[PROBE_TELEMETRY_TEXT_LEN];
  char l2[PROBE_TELEMETRY_TEXT_LEN];
  const DriverRequirements &r = d.getRequirements();
  CHECK(same(d.getName(), "HC-SR04 Ultrasonic"));
  CHECK(r.topology == ProbeTopology::FourWirePowered && r.excitation == ExcitationPolicy::ProtectedActive);
  CHECK((r.backendCapabilities & kBackendSupply5V) != 0);  // a 5 V module, never GPIO-powered
  CHECK(same(r.signalLabel[0], "TRG") && same(r.signalLabel[1], "ECH"));
  CHECK(r.powerStabilizeMs == ULTRASONIC_POWER_STABILIZE_MS && r.minAcquisitionIntervalMs >= 70);
  CHECK(r.sampleIntervalMs >= r.minAcquisitionIntervalMs);

  // Wrong topology and bad buffers: refused without touching a pin.
  size_t quiet = mock::eventCount;
  CHECK(!d.probe(poweredContext(0)) && same(d.getFailureReason(), "topology-mismatch"));
  CHECK(!d.readSample(nullptr, l2, sizeof l2) && same(d.getFailureReason(), "invalid-buffer"));
  memset(l1, 'x', sizeof l1);
  CHECK(!d.readSample(l1, l2, 0) && l1[0] == 'x');
  CHECK(!d.readSample(l1, l2, sizeof l1) && same(d.getFailureReason(), "not-probed"));
  CHECK(l1[0] == '\0' && l2[0] == '\0');
  CHECK(mock::eventCount == quiet && mock::attachCalls == 0);

  // probe(): two consecutive fresh echoes, triggers at least 70 ms apart,
  // Pending in between. Only TRIG is driven; the VCC and GND GPIOs stay
  // plain INPUT (the backend powers the part).
  sonar.attached = true;
  sonar.wiring = 0;  // A VCC, B TRIG, C ECHO, D GND
  powerSonarByHand();
  const ProbeContext ctx = fourWireContext(0);
  int pending = 0;
  while (!d.probe(ctx) && d.getOperationStatus() == DriverOperationStatus::Pending && pending < 1000) {
    pending++;
    advanceTo(mock::nowUs + 1000);
  }
  CHECK(d.getOperationStatus() == DriverOperationStatus::Succeeded);
  CHECK(sonar.triggers == ULTRASONIC_CONFIRM_SAMPLES && sonar.pulses == ULTRASONIC_CONFIRM_SAMPLES);
  CHECK(sonar.triggerUs[1] - sonar.triggerUs[0] >= ULTRASONIC_MIN_INTERVAL_MS * 1000ULL);
  CHECK(pending >= static_cast<int>(ULTRASONIC_MIN_INTERVAL_MS));
  CHECK(sonar.minPulseUs >= ULTRASONIC_TRIGGER_PULSE_US && sonar.minPulseUs < 30);  // bounded busy wait
  CHECK(mock::pins[ctx.fourWire.sig1].mode == OUTPUT && mock::pins[ctx.fourWire.sig1].latch == LOW);
  CHECK(mock::pins[ctx.fourWire.sig2].mode == INPUT && mock::isr[ctx.fourWire.sig2].fn != nullptr);
  CHECK(mock::isr[ctx.fourWire.sig2].mode == CHANGE);
  CHECK(mock::pins[ctx.fourWire.vcc].mode == INPUT && mock::pins[ctx.fourWire.gnd].mode == INPUT);
  CHECK(mock::criticalDepth == 0 && mock::criticalMaxUs < 5);  // only field copies are locked

  // readSample(): a fresh echo every time, never a cached one, after the
  // trigger spacing.
  CHECK(!d.readSample(l1, l2, sizeof l1));
  CHECK(d.getOperationStatus() == DriverOperationStatus::Pending && sonar.pulses == 2);  // spacing
  CHECK(l1[0] == '\0' && l2[0] == '\0');
  CHECK(driveSample(d, l1, l2, sizeof l1));
  CHECK(same(l1, "Dist: 123.4 cm") && same(l2, "Echo: 7157 us") && sonar.triggers == 3);
  sonar.widthUs = 1160;  // 20.0 cm
  advanceTo(mock::nowUs + ULTRASONIC_SAMPLE_INTERVAL_MS * 1000ULL);
  CHECK(driveSample(d, l1, l2, sizeof l1));
  CHECK(same(l1, "Dist: 20.0 cm") && same(l2, "Echo: 1160 us") && sonar.triggers == 4);

  // An echo that completed in time but is processed long after the 25 ms
  // deadline is accepted: its own falling-edge timestamp is what counts.
  sonar.widthUs = 7157;
  advanceTo(mock::nowUs + 100000);
  CHECK(!d.readSample(l1, l2, sizeof l1) && sonar.triggers == 5);
  advanceTo(mock::nowUs + 60000);  // edges at +0.46 and +7.6 ms; next call at +60 ms
  CHECK(d.readSample(l1, l2, sizeof l1) && same(l2, "Echo: 7157 us"));

  // Edges while disarmed (between transactions) are ignored: a stale pulse
  // never completes or corrupts the next measurement.
  const uint8_t echoPin = ctx.fourWire.sig2;
  sonar.level = HIGH;
  CHECK(mock::fireInterrupt(echoPin));
  sonar.level = LOW;
  CHECK(mock::fireInterrupt(echoPin));
  sonar.widthUs = 2900;  // 50.0 cm
  advanceTo(mock::nowUs + 100000);
  CHECK(driveSample(d, l1, l2, sizeof l1) && same(l1, "Dist: 50.0 cm"));
  for (int i = 1; i < sonar.triggers; i++) {
    CHECK(sonar.triggerUs[i] - sonar.triggerUs[i - 1] >= ULTRASONIC_MIN_INTERVAL_MS * 1000ULL);
  }

  // Cancellation: reset() mid-echo detaches the capture and releases TRIG;
  // the falling edge then reaches nothing and nothing is published.
  advanceTo(mock::nowUs + 100000);
  CHECK(!d.readSample(l1, l2, sizeof l1));
  advanceTo(mock::nowUs + 1000);  // the rising edge has been captured
  CHECK(sonar.level == HIGH);
  d.reset();
  CHECK(d.getOperationStatus() == DriverOperationStatus::Idle && !anyProbeIsrAttached());
  CHECK(allProbesPlainInput());
  const int isrCalls = mock::isrCalls;
  advanceTo(mock::nowUs + 20000);  // the falling edge arrives late
  CHECK(sonar.level == LOW && mock::isrCalls == isrCalls);
  CHECK(!d.readSample(l1, l2, sizeof l1) && same(d.getFailureReason(), "not-probed"));
  quiet = mock::eventCount;
  d.reset();
  d.reset();
  CHECK(mock::eventCount == quiet);

  // Changing arguments while an operation is pending is refused.
  advanceTo(mock::nowUs + 100000);
  CHECK(!d.probe(ctx) && d.getOperationStatus() == DriverOperationStatus::Pending);
  CHECK(!d.probe(fourWireContext(1)) && same(d.getFailureReason(), "contract-violation"));
  CHECK(!anyProbeIsrAttached() && allProbesPlainInput());

  // A new mapping is a new binding: the old ECHO interrupt is detached and
  // confirmation starts again from zero.
  d.reset();
  advanceTo(mock::nowUs + 100000);
  CHECK(driveProbe(d, ctx));
  int triggers = sonar.triggers;
  CHECK(!driveProbe(d, fourWireContext(1)));  // A VCC, B TRIG, D ECHO, C GND: nothing on D
  CHECK(same(d.getFailureReason(), "no-echo") && sonar.triggers == triggers + 1);
  CHECK(mock::isr[gpioOf(2)].fn == nullptr && !anyProbeIsrAttached());
  advanceTo(mock::nowUs + 100000);
  triggers = sonar.triggers;
  CHECK(driveProbe(d, ctx) && sonar.triggers == triggers + ULTRASONIC_CONFIRM_SAMPLES);
  d.reset();
  CHECK(mock::criticalDepth == 0);

  // Failure modes. Each leaves TRIG released and the capture detached.
  CHECK(same(sampleFailure([] { sonar.stuckHigh = true; }), "echo-stuck-high") && faultPulses == 0);
  CHECK(same(sampleFailure([] { sonar.respond = false; }), "no-echo"));  // no echo or stuck LOW
  CHECK(faultPulses == 1 && faultMs >= ULTRASONIC_ECHO_DEADLINE_US / 1000 && faultMs <= 27);
  CHECK(same(sampleFailure([] { sonar.widthUs = 30000; }), "echo-timeout"));  // rose, never fell in time
  CHECK(faultMs <= 27);
  CHECK(same(sampleFailure([] { sonar.widthUs = 100; }), "out-of-range"));    // 1.7 cm
  CHECK(same(sampleFailure([] { sonar.widthUs = 20000; }), "out-of-range"));  // 344.8 cm
  CHECK(same(sampleFailure([] {
               sonar.coupledToTrig = true;  // ECHO follows TRIG during the pulse
               sonar.respond = false;
             }),
             "pre-trigger-echo"));
  CHECK(same(sampleFailure([] { sonar.riseDelayUs = 3000; },
                           [] { mock::fireInterrupt(sonarGpio(NodeRole::Sig2)); }),  // a falling edge first
             "malformed-echo"));
  CHECK(same(sampleFailure([] {},
                           [] {
                             advanceTo(mock::nowUs + 1000);                  // the rise is captured
                             mock::fireInterrupt(sonarGpio(NodeRole::Sig2));  // HIGH again: a missed fall
                           }),
             "malformed-echo"));
  CHECK(sampleFailure([] {}) == nullptr);  // the baseline really succeeds

  // micros() rolls over between the rising and falling edge (a continuous
  // clock that starts 3 ms before the wrap).
  useDrivers();
  resetWorld((1ULL << 32) - 3000ULL, kAllBackendCaps);
  sonar.attached = true;
  powerSonarByHand();
  UltrasonicDriver w;
  CHECK(driveProbe(w, ctx));
  CHECK(sonar.triggerUs[0] < (1ULL << 32) && sonar.triggerUs[0] + 460 + 7157 > (1ULL << 32));
  advanceTo(mock::nowUs + 100000);
  CHECK(driveSample(w, l1, l2, sizeof l1) && same(l2, "Echo: 7157 us"));
  w.reset();
}

// After an ultrasonic session and its reset, the same pins serve the passive
// ADC and the DHT protocol: no interrupt or drive is left behind.
static void testUltrasonicHandoff() {
  useDrivers();
  resetWorld(1000000ULL, kAllBackendCaps);
  sonar.attached = true;
  powerSonarByHand();
  UltrasonicDriver ud;
  CHECK(driveProbe(ud, fourWireContext(0)));
  CHECK(anyProbeIsrAttached());
  resetTestPins();  // the engine's teardown: GPIOs, then the backend
  ud.reset();
  CHECK(!anyProbeIsrAttached() && allProbesPlainInput() && !backend.connected);
  CHECK(backend.signalsDrivenAtOff == 0);  // TRIG was released before the supply
  sonar.attached = false;

  PassiveDriver pd;
  connect(1, 2, 10000);  // B-C: the former TRIG and ECHO nodes
  const int adcBefore = adcWithIsrAttached;
  CHECK(driveProbe(pd, pairContext(1)));
  CHECK(same(pd.getName(), "Resistive Component") && adcWithIsrAttached == adcBefore);
  pd.reset();
  netReset();

  Dht11Driver dd;
  sensor.attached = true;
  sensor.wiring = PROBE_PERMUTATIONS[0];  // DATA on B, the former TRIG
  powerByHand(PROBE_PERMUTATIONS[0]);
  CHECK(driveProbe(dd, poweredContext(0)) && sensor.responses == 1);
  dd.reset();
  resetTestPins();
}

#if PINDEX_POWERED_PROBING_ENABLED

// ---- DHT11 through the engine (Phase 2 behaviour, preserved) ---------------

static void resetWithDht(uint64_t startUs = 1000000ULL) {
  useDrivers(&dht);
  resetWorld(startUs);
}

static bool detectWith(int wiring) {
  resetWithDht();
  sensor.attached = true;
  sensor.wiring = PROBE_PERMUTATIONS[wiring];
  return runUntil([] { return phase() == ProbePhase::Detected; }, 40000);
}

static void testEmptySocketScan() {
  resetWithDht();
  const uint64_t t0 = mock::nowUs;
  CHECK(runUntil([] { return phase() == ProbePhase::NoComponent; }, 60000));
  CHECK(same(probeStatus().lastError, dhtErrorName(DhtError::NoResponse)));
  CHECK(probeStatus().precheck == PrecheckResult::NoObviousCoupling);
  CHECK(!probeStatus().hasMapping && !probeStatus().readingValid);
  CHECK(probeStatus().componentName == nullptr && !probeStatus().measurementUnavailable);
  CHECK(probeStatus().hardwareRequired == nullptr);
  CHECK(allProbesPlainInput());

  // Latency is dominated by per-mapping stabilization (~1.47 s each, 24).
  const uint64_t scanMs = (mock::nowUs - t0) / 1000;
  CHECK(scanMs >= PROBE_PERMUTATION_COUNT * DHT_POWER_STABILIZE_MS &&
        scanMs <= PROBE_PERMUTATION_COUNT * 1600);

  // All 24 three-wire mappings in table order (the original six first), each
  // leaving its fourth node plain INPUT throughout.
  const SessionLog log = collectSessions();
  CHECK(log.violations == 0);
  CHECK(log.count == PROBE_PERMUTATION_COUNT);
  for (int i = 0; i < log.count && i < PROBE_PERMUTATION_COUNT; i++) {
    const Session &x = log.s[i];
    CHECK(sessionUses(x, PROBE_PERMUTATIONS[i]));  // exact table order
    CHECK(unusedStayedInput(x, PROBE_PERMUTATIONS[i]));
    CHECK(x.endUs != UINT64_MAX && x.starts == 1);
    CHECK(x.firstStartUs - x.startUs >= (DHT_POWER_STABILIZE_MS - 1) * 1000);
    if (i > 0) {  // torn down, then the power-off interval, before the next one
      CHECK(x.startUs - log.s[i - 1].endUs >= (PROBE_POWER_OFF_MS - 1) * 1000);
    }
  }

  // Probes stay INPUT for the rescan interval, then discovery restarts.
  runMs(PROBE_RESCAN_INTERVAL_MS - 50);
  CHECK(phase() == ProbePhase::NoComponent);
  CHECK(collectSessions().count == PROBE_PERMUTATION_COUNT);
  runMs(100);
  CHECK(phase() == ProbePhase::Scanning && probeStatus().scanIndex == 1);
  CHECK(probeStatus().scanTotal == PROBE_PERMUTATION_COUNT);
}

static void testDetectionNeedsTwoFrames() {
  resetWithDht();
  sensor.attached = true;
  sensor.wiring = PROBE_PERMUTATIONS[3];  // B = VCC, C = DATA, A = GND
  CHECK(runUntil([] { return phase() == ProbePhase::Confirming; }, 20000));
  CHECK(sensor.responses == 1);
  CHECK(probeStatus().hasMapping && !probeStatus().detected);
  CHECK(!probeStatus().readingValid);  // probe() proves identity; telemetry comes from samples
  CHECK(same(probeStatus().componentName, "DHT11 Temp & Hum"));

  CHECK(runUntil([] { return phase() == ProbePhase::Detected; }, 3000));
  CHECK(sensor.responses == 2);  // two genuine frames
  CHECK(same(probeStatus().telemetry1, "T: 25*C  |  H: 60%"));
  CHECK(probeStatus().telemetry2[0] == '\0');
  CHECK(!probeStatus().readingStale && probeStatus().consecutiveFailures == 0);
  CHECK(probeStatus().lastError == nullptr);
  char first[PROBE_MAPPING_TEXT_LEN];
  char second[PROBE_MAPPING_TEXT_LEN];
  formatProbeMapping(probeStatus().mapping, probeStatus().signalLabel, first, second, sizeof first);
  CHECK(same(first, "A:GND B:VCC") && same(second, "C:DAT D:---"));

  // Permutations 1-3 were torn down; 4 stayed powered for both frames. D
  // (unused by all four) never left plain INPUT while anything was powered.
  SessionLog log = collectSessions();
  CHECK(log.violations == 0);
  CHECK(log.count == 4);
  for (int i = 0; i < log.count && i < 4; i++) CHECK((log.s[i].usedMask & (1u << 3)) == 0);
  CHECK(log.s[3].endUs == UINT64_MAX && log.s[3].starts == 2);
  CHECK(log.startCount >= 2);
  CHECK(log.startTimes[log.startCount - 1] - log.startTimes[log.startCount - 2] >=
        (DHT_READ_INTERVAL_MS - 1) * 1000);

  // Monitoring reads every DHT_READ_INTERVAL_MS and powers nothing else.
  const uint8_t next[5] = {61, 0, 26, 0, 87};
  memcpy(sensor.frame, next, 5);
  const uint32_t lastMs = probeStatus().lastReadingMs;
  const uint32_t rev = probeStatus().revision;
  runMs(DHT_READ_INTERVAL_MS + 100);
  CHECK(same(probeStatus().telemetry1, "T: 26*C  |  H: 61%"));
  CHECK(probeStatus().lastReadingMs - lastMs >= DHT_READ_INTERVAL_MS - 50);
  CHECK(probeStatus().revision != rev);
  log = collectSessions();
  CHECK(log.count == 4 && log.violations == 0);

  // Identical readings are not a display change.
  const uint32_t quietRev = probeStatus().revision;
  runMs(2 * DHT_READ_INTERVAL_MS);
  CHECK(probeStatus().revision == quietRev);
  CHECK(phase() == ProbePhase::Detected);
}

// DHT11 on a mapping that uses D (B = VCC, A = DATA, D = GND: entry 9):
// found after the eight before it, with C never touched.
static void testDhtOnNodeD() {
  CHECK(detectWith(8));
  char first[PROBE_MAPPING_TEXT_LEN];
  char second[PROBE_MAPPING_TEXT_LEN];
  formatProbeMapping(probeStatus().mapping, probeStatus().signalLabel, first, second, sizeof first);
  CHECK(same(first, "A:DAT B:VCC") && same(second, "C:--- D:GND"));
  CHECK(same(probeStatus().telemetry1, "T: 25*C  |  H: 60%"));
  const SessionLog log = collectSessions();
  CHECK(log.violations == 0 && log.count == 9);
  for (int i = 0; i < log.count && i < 9; i++) {
    CHECK(sessionUses(log.s[i], PROBE_PERMUTATIONS[i]) && unusedStayedInput(log.s[i], PROBE_PERMUTATIONS[i]));
  }
}

static void testStaleThenRecovery() {
  CHECK(detectWith(3));
  sensor.respond = false;  // e.g. a bad contact for one read
  CHECK(runUntil([] { return probeStatus().consecutiveFailures == 1; }, 3000));
  CHECK(phase() == ProbePhase::Retrying);
  CHECK(probeStatus().readingStale && probeStatus().readingValid && probeStatus().detected);
  CHECK(same(probeStatus().telemetry1, "T: 25*C  |  H: 60%"));  // kept, marked stale
  CHECK(same(probeStatus().lastError, dhtErrorName(DhtError::NoResponse)));
  CHECK(allProbesPlainInput());  // torn down at once

  // The retry re-probes the saved driver; that proves the link but is not
  // a published sample, so the telemetry stays stale until the next one.
  sensor.respond = true;
  CHECK(runUntil([] { return probeStatus().consecutiveFailures == 0; }, 3000));
  CHECK(phase() == ProbePhase::Retrying && probeStatus().readingStale);
  CHECK(runUntil([] { return phase() == ProbePhase::Detected; }, 3000));
  CHECK(!probeStatus().readingStale);

  const SessionLog log = collectSessions();
  CHECK(log.violations == 0);
  CHECK(log.count == 5);
  CHECK(sessionUses(log.s[4], PROBE_PERMUTATIONS[3]));
  CHECK(log.s[4].starts == 2);  // re-probe, then a new sample
  CHECK(log.s[4].startUs - log.s[3].endUs >= (PROBE_RETRY_COOLDOWN_MS - 1) * 1000);
  CHECK(log.s[4].firstStartUs - log.s[4].startUs >= (DHT_POWER_STABILIZE_MS - 1) * 1000);
  for (int i = 1; i < log.startCount; i++) {
    if (log.startTimes[i - 1] >= log.s[3].startUs) {
      CHECK(log.startTimes[i] - log.startTimes[i - 1] >= (DHT_MIN_START_INTERVAL_MS - 1) * 1000);
    }
  }
}

static void testRemovalAfterThreeFailures() {
  CHECK(detectWith(3));
  const int sessionsBefore = collectSessions().count;
  sensor.attached = false;  // component pulled out between reads
  const uint64_t removedUs = mock::nowUs;

  CHECK(runUntil([] { return probeStatus().consecutiveFailures == 1; }, 3000));
  CHECK(phase() == ProbePhase::Retrying && probeStatus().readingStale);
  CHECK(runUntil([] { return probeStatus().consecutiveFailures == 2; }, 4000));
  CHECK(probeStatus().hasMapping);
  CHECK(runUntil([] { return phase() == ProbePhase::Idle; }, 4000));
  CHECK(!probeStatus().hasMapping && !probeStatus().detected);
  CHECK(!probeStatus().readingValid && !probeStatus().readingStale);
  CHECK(probeStatus().componentName == nullptr && probeStatus().telemetry1[0] == '\0');
  CHECK(allProbesPlainInput());
  CHECK((mock::nowUs - removedUs) / 1000 <= DHT_READ_INTERVAL_MS + 2 * 2300);

  SessionLog log = collectSessions();
  CHECK(log.violations == 0);
  CHECK(log.count == sessionsBefore + 2);  // two retries, saved mapping only
  for (int i = sessionsBefore - 1; i < log.count; i++) {
    CHECK(sessionUses(log.s[i], PROBE_PERMUTATIONS[3]) && log.s[i].endUs != UINT64_MAX);
  }

  runMs(PROBE_RESCAN_INTERVAL_MS - 50);
  CHECK(phase() == ProbePhase::Idle);
  CHECK(collectSessions().count == log.count);
  runMs(100);
  CHECK(phase() == ProbePhase::Scanning && probeStatus().scanIndex == 1);
}

// Runs permutation 1 against a misbehaving sensor wired as permutation 1 and
// returns the reason that rejected it.
static const char *firstAttemptError(void (*configure)()) {
  resetWithDht();
  sensor.attached = true;
  sensor.wiring = PROBE_PERMUTATIONS[0];
  configure();
  CHECK(runUntil([] { return probeStatus().scanIndex == 2; }, 3000));
  CHECK(!probeStatus().hasMapping);
  CHECK(allProbesPlainInput());
  CHECK(collectSessions().violations == 0);
  return probeStatus().lastError;
}

static void testCaptureErrors() {
  CHECK(same(firstAttemptError([] { sensor.stuckLow = true; }), dhtErrorName(DhtError::StuckLow)));
  CHECK(same(firstAttemptError([] { sensor.respond = false; }),  // stuck high
             dhtErrorName(DhtError::NoResponse)));
  CHECK(same(firstAttemptError([] { sensor.ackLowUs = 20; }), dhtErrorName(DhtError::BadAckTiming)));
  CHECK(same(firstAttemptError([] { sensor.ackLowUs = 200; }), dhtErrorName(DhtError::AckTimeout)));
  CHECK(same(firstAttemptError([] { sensor.bitsSent = 20; }),  // truncated
             dhtErrorName(DhtError::BitTimeout)));
  CHECK(same(firstAttemptError([] { sensor.oneHighUs = 110; }), dhtErrorName(DhtError::BadBitTiming)));
  CHECK(same(firstAttemptError([] { sensor.frame[4] ^= 1; }), dhtErrorName(DhtError::Checksum)));
  CHECK(same(firstAttemptError([] {
               const uint8_t f[5] = {60, 5, 25, 0, 90};
               memcpy(sensor.frame, f, 5);
             }),
             dhtErrorName(DhtError::UnsupportedEncoding)));

  // Every phase in range but the frame too slow overall: the whole-frame
  // deadline still ends the capture, bounding the time interrupts are masked.
  CHECK(same(firstAttemptError([] {
               sensor.bitLowUs = 80;
               sensor.oneHighUs = 100;
               memset(sensor.frame, 0xFF, 5);
             }),
             dhtErrorName(DhtError::BitTimeout)));
  CHECK(mock::criticalMaxUs > 0 && mock::criticalMaxUs <= DHT_FRAME_DEADLINE_US + 10);
}

// Coupling routes instead of latching a fault; powered drive never crosses
// a coupled path; noise is retried, then "unknown".
static void testPrecheckRouting() {
  resetWithDht();
  connect(0, 1, 1.0);  // simulated A-B short
  CHECK(runUntil([] { return phase() == ProbePhase::NoComponent; }, 60000));
  CHECK(probeStatus().precheck == PrecheckResult::CouplingObserved);
  CHECK(probeStatus().coupledPairs == probePairBit(0, 1));
  SessionLog log = collectSessions();
  CHECK(log.violations == 0);
  CHECK(poweredSessions(log) == 12);  // only the mappings with VCC on C or D (6 + 6)
  for (int i = 0; i < log.count; i++) CHECK(log.s[i].vcc == 2 || log.s[i].vcc == 3);
  CHECK(!probeRequestRetry());  // nothing latched
  CHECK(allProbesPlainInput());

  // Inconsistent readings: bounded retries, then unknown with nothing powered.
  resetWithDht();
  flakyA = true;
  CHECK(runUntil([] { return phase() == ProbePhase::NoComponent; }, 2000));
  CHECK(probeStatus().precheck == PrecheckResult::InconsistentReadings);
  CHECK(same(probeStatus().lastError, "precheck-inconsistent"));
  CHECK(flakyReads == 2 * (1 + PRECHECK_MAX_RETRIES));  // inconsistency seen on sample 2 of each pass
  CHECK(collectSessions().count == 0);

  // Coupling that appears during a retry of the saved DHT mapping blocks
  // power on that mapping (VCC on B, coupled to A); it is never powered again.
  CHECK(detectWith(3));
  sensor.respond = false;
  CHECK(runUntil([] { return probeStatus().consecutiveFailures == 1; }, 3000));
  const int sessionsAtFailure = collectSessions().count;
  connect(0, 1, 1.0);
  CHECK(runUntil([] { return phase() == ProbePhase::Idle; }, 10000));
  CHECK(same(probeStatus().lastError, "blocked-by-precheck"));
  CHECK(collectSessions().count == sessionsAtFailure);
  CHECK(!probeStatus().hasMapping && probeStatus().componentName == nullptr);
  CHECK(allProbesPlainInput());
}

static void testRollover() {
  // millis() wraps ~600 ms in, during the first power-up stabilization.
  resetWithDht((static_cast<uint64_t>(0xFFFFFFFFu) - 600) * 1000);
  sensor.attached = true;
  sensor.wiring = PROBE_PERMUTATIONS[0];
  CHECK(runUntil([] { return phase() == ProbePhase::Detected; }, 5000));
  const SessionLog log = collectSessions();
  CHECK(log.count == 1 && log.violations == 0);
  CHECK(log.s[0].firstStartUs - log.s[0].startUs >= (DHT_POWER_STABILIZE_MS - 1) * 1000);
  CHECK(log.s[0].firstStartUs - log.s[0].startUs <= (DHT_POWER_STABILIZE_MS + 2) * 1000);
  CHECK(millis() < 10000);  // the counter really wrapped

  // micros() wraps inside every capture.
  resetWithDht();
  sensor.attached = true;
  sensor.wiring = PROBE_PERMUTATIONS[0];
  sensor.wrapMicrosAtRelease = true;
  CHECK(runUntil([] { return phase() == ProbePhase::Detected; }, 5000));
  CHECK(same(probeStatus().telemetry1, "T: 25*C  |  H: 60%"));
}

// ---- Engine architecture with powered fake drivers -------------------------

static void testPermutationMajorDriverOrder() {
  FakeDriver a(1, "Fake A");
  FakeDriver b(2, "Fake B");  // both always fail
  useDrivers(&a, &b);
  resetWorld();
  CHECK(runUntil([] { return phase() == ProbePhase::NoComponent; }, 120000));
  CHECK(same(probeStatus().lastError, "fake-failed"));
  CHECK(allProbesPlainInput());

  CHECK(attemptCount == 2 * PROBE_PERMUTATION_COUNT);
  for (int i = 0; i < attemptCount; i++) {  // permutation-major, registry order inside
    CHECK(attempts[i].slot == i / 2 && attempts[i].driver == 1 + i % 2 && !attempts[i].sample);
  }
  CHECK(a.sampleStarts == 0 && b.sampleStarts == 0);
  CHECK(a.unpoweredStarts == 0 && b.unpoweredStarts == 0);  // the engine powered each attempt
  CHECK(a.resets >= 1 + PROBE_PERMUTATION_COUNT && b.resets >= 1 + PROBE_PERMUTATION_COUNT);
  CHECK(a.unsafeResets == 0 && b.unsafeResets == 0);  // pins were safe before every reset

  const SessionLog log = collectSessions();
  CHECK(log.violations == 0);
  CHECK(log.count == attemptCount);
  for (int i = 0; i < log.count && i < attemptCount; i++) {
    CHECK(sessionPowers(log.s[i], PROBE_PERMUTATIONS[i / 2]));
    CHECK(attempts[i].us - log.s[i].startUs >= (a.req.powerStabilizeMs - 1) * 1000ULL);
    if (i > 0) CHECK(log.s[i].startUs - log.s[i - 1].endUs >= (PROBE_POWER_OFF_MS - 1) * 1000);
  }

  runMs(PROBE_RESCAN_INTERVAL_MS - 50);  // rescan interval preserved
  CHECK(phase() == ProbePhase::NoComponent && attemptCount == 2 * PROBE_PERMUTATION_COUNT);
}

static void testLaterDriverAndConfirmation() {
  FakeDriver a(1, "Fake A");
  FakeDriver b(2, "Fake B");
  b.acceptSlot = 0;
  useDrivers(&a, &b);
  resetWorld();
  CHECK(runUntil([] { return phase() == ProbePhase::Confirming; }, 10000));
  CHECK(same(probeStatus().componentName, "Fake B"));
  CHECK(a.probeStarts == 1 && b.probeStarts == 1);
  CHECK(sameRoles(probeStatus().mapping, PROBE_PERMUTATIONS[0]));
  CHECK(!probeStatus().detected && !probeStatus().readingValid);

  // Reading the snapshot (what the OLED does) never acquires or confirms.
  const int callsBefore = b.calls;
  for (int i = 0; i < 1000; i++) {
    char m1[PROBE_MAPPING_TEXT_LEN];
    char m2[PROBE_MAPPING_TEXT_LEN];
    formatProbeMapping(probeStatus().mapping, probeStatus().signalLabel, m1, m2, sizeof m1);
  }
  CHECK(b.calls == callsBefore && !probeStatus().detected);

  // Confirmation needs genuinely new acquisitions: probe + new sample(s).
  CHECK(runUntil([] { return phase() == ProbePhase::Detected; }, 2000));
  CHECK(b.probeStarts == 1 && b.sampleStarts == PROBE_CONFIRM_FRAMES - 1);
  CHECK(same(probeStatus().telemetry1, "Fake B #1") && same(probeStatus().telemetry2, "second line"));

  // Monitoring delegates only to the active driver, at its sample interval.
  const int aCalls = a.calls;
  runMs(5 * b.req.sampleIntervalMs + 50);
  CHECK(a.calls == aCalls);
  CHECK(b.sampleStarts == PROBE_CONFIRM_FRAMES - 1 + 5 && b.probeStarts == 1);
  CHECK(same(probeStatus().telemetry1, "Fake B #6"));
  CHECK(b.unpoweredStarts == 0);
}

static void testPendingIsNotFailure() {
  FakeDriver a(1, "Fake A");
  FakeDriver b(2, "Fake B");
  a.acceptSlot = 0;
  a.pendingCalls = 200;  // well inside a.req.operationTimeoutMs
  useDrivers(&a, &b);
  resetWorld();
  CHECK(runUntil([&] { return a.probeStarts == 1; }, 2000));
  runMs(150);
  CHECK(a.getOperationStatus() == DriverOperationStatus::Pending);
  CHECK(a.probeStarts == 1 && b.calls == 0);  // same operation; registry not advanced
  CHECK(probeStatus().scanIndex == 1 && probeStatus().consecutiveFailures == 0);
  CHECK(probeStatus().lastError == nullptr);
  CHECK(runUntil([] { return phase() == ProbePhase::Confirming; }, 1000));
  CHECK(runUntil([] { return phase() == ProbePhase::Detected; }, 3000));  // pending sample too
  CHECK(b.calls == 0 && probeStatus().consecutiveFailures == 0);

  // A driver that never completes is abandoned after its own timeout, torn
  // down and reset, and the next driver is tried.
  FakeDriver stuck(1, "Fake Stuck");
  FakeDriver next(2, "Fake Next");
  stuck.acceptSlot = 0;
  stuck.pendingCalls = 1000000;
  next.acceptSlot = 0;
  useDrivers(&stuck, &next);
  attemptCount = 0;
  resetWorld();
  CHECK(runUntil([&] { return stuck.resets == 2; }, 3000));  // init + timeout
  CHECK(same(probeStatus().lastError, "operation-timeout") && allProbesPlainInput());
  CHECK(attempts[0].driver == 1 && attemptCount == 1);
  CHECK(mock::nowUs / 1000 - attempts[0].us / 1000 >= stuck.req.operationTimeoutMs);
  CHECK(runUntil([] { return phase() == ProbePhase::Confirming; }, 3000));
  CHECK(same(probeStatus().componentName, "Fake Next") && stuck.unsafeResets == 0);
}

static void testRequirementsDriveScheduling() {
  FakeDriver a(1, "Fake A");
  a.acceptSlot = 0;
  a.req.powerStabilizeMs = 250;
  a.req.minAcquisitionIntervalMs = 600;
  a.req.sampleIntervalMs = 200;  // below the minimum: the minimum wins
  useDrivers(&a);
  attemptCount = 0;
  resetWorld();
  CHECK(runUntil([] { return phase() == ProbePhase::Detected; }, 5000));
  runMs(3000);
  const SessionLog log = collectSessions();
  CHECK(log.count == 1 && log.violations == 0);
  CHECK(attemptCount >= 4 && !attempts[0].sample);
  const uint64_t stabilizeUs = attempts[0].us - log.s[0].startUs;
  CHECK(stabilizeUs >= 249000 && stabilizeUs <= 252000);
  for (int i = 1; i < attemptCount; i++) {
    const uint64_t gapUs = attempts[i].us - attempts[i - 1].us;
    CHECK(attempts[i].sample && gapUs >= 599000 && gapUs <= 601000);
  }
}

static void testRemovalClearsActiveDriver() {
  FakeDriver a(1, "Fake A");
  FakeDriver b(2, "Fake B");
  b.acceptSlot = 2;
  useDrivers(&a, &b);
  attemptCount = 0;
  resetWorld();
  CHECK(runUntil([] { return phase() == ProbePhase::Detected; }, 20000));
  const int detectedAt = attemptCount;
  char shown[PROBE_TELEMETRY_TEXT_LEN];
  snprintf(shown, sizeof shown, "%s", probeStatus().telemetry1);

  b.acceptSlot = -1;  // component pulled out
  b.samplesSucceed = false;
  CHECK(runUntil([] { return probeStatus().consecutiveFailures == 1; }, 2000));
  CHECK(phase() == ProbePhase::Retrying && same(probeStatus().telemetry1, shown));
  CHECK(runUntil([] { return phase() == ProbePhase::Idle; }, 10000));
  CHECK(probeStatus().componentName == nullptr && !probeStatus().hasMapping);
  CHECK(!probeStatus().detected && !probeStatus().readingValid);
  CHECK(probeStatus().telemetry1[0] == '\0' && probeStatus().telemetry2[0] == '\0');
  CHECK(allProbesPlainInput() && a.unsafeResets == 0 && b.unsafeResets == 0);
  for (int i = detectedAt; i < attemptCount; i++) {  // recovery used only the saved driver
    CHECK(attempts[i].driver == 2 && attempts[i].slot == 2);
  }

  const int before = attemptCount;
  runMs(PROBE_RESCAN_INTERVAL_MS + 1000);
  CHECK(attemptCount > before && attempts[before].driver == 1 && attempts[before].slot == 0);
}

static void testEmptyRegistry() {
  useDrivers();
  resetWorld();
  runMs(20000);
  CHECK(phase() == ProbePhase::NoComponent);
  for (size_t i = 0; i < mock::eventCount; i++) {
    if (nodeOf(mock::events[i].pin) >= 0) CHECK(mock::events[i].mode == INPUT);  // no pulls, no power
  }
  CHECK(!probeRequestRetry());
}

// ---- Four-wire through the engine, on the fake protected backend ------------

static void testUltrasonicDetection() {
  useDrivers(&ultrasonic);
  resetWorld(1000000ULL, kAllBackendCaps);
  sonar.attached = true;
  sonar.wiring = 0;  // A VCC, B TRIG, C ECHO, D GND
  CHECK(runUntil([] { return phase() == ProbePhase::Scanning; }, 100));
  CHECK(probeStatus().scanTotal == PROBE_FOUR_WIRE_COUNT && probeStatus().hardwareRequired == nullptr);
  CHECK(runUntil([] { return phase() == ProbePhase::Confirming; }, 2000));
  CHECK(sonar.triggers == ULTRASONIC_CONFIRM_SAMPLES && !probeStatus().readingValid);
  CHECK(sonar.triggerUs[0] - backend.connectedAtUs >= (ULTRASONIC_POWER_STABILIZE_MS - 1) * 1000ULL);
  CHECK(runUntil([] { return phase() == ProbePhase::Detected; }, 1000));
  CHECK(sonar.triggers == ULTRASONIC_CONFIRM_SAMPLES + 1);  // + the engine's confirming sample
  CHECK(same(probeStatus().componentName, "HC-SR04 Ultrasonic"));
  CHECK(same(probeStatus().telemetry1, "Dist: 123.4 cm") && same(probeStatus().telemetry2, "Echo: 7157 us"));
  char first[PROBE_MAPPING_TEXT_LEN];
  char second[PROBE_MAPPING_TEXT_LEN];
  formatProbeMapping(probeStatus().mapping, probeStatus().signalLabel, first, second, sizeof first);
  CHECK(same(first, "A:VCC B:TRG") && same(second, "C:ECH D:GND"));
  CHECK(backend.connected && backend.vcc == 0 && backend.sig1 == 1 && backend.sig2 == 2 && backend.gnd == 3);
  CHECK(backend.connects == 1 && backend.violations == 0);
  CHECK(mock::pins[gpioOf(0)].mode == INPUT && mock::pins[gpioOf(3)].mode == INPUT);  // supply: backend only

  // Monitoring: a fresh echo every sample interval, triggers never closer
  // than the minimum interval; an unchanged reading is not a redraw.
  sonar.widthUs = 2900;  // 50.0 cm
  CHECK(waitSamples(2));
  CHECK(same(probeStatus().telemetry1, "Dist: 50.0 cm") && same(probeStatus().telemetry2, "Echo: 2900 us"));
  const uint32_t rev = probeStatus().revision;
  const int triggersBefore = sonar.triggers;
  runMs(2000);
  CHECK(probeStatus().revision == rev);
  CHECK(sonar.triggers - triggersBefore >= static_cast<int>(2000 / ULTRASONIC_SAMPLE_INTERVAL_MS) - 1);
  for (int i = 1; i < sonar.triggers && i < 128; i++) {
    CHECK(sonar.triggerUs[i] - sonar.triggerUs[i - 1] >= ULTRASONIC_MIN_INTERVAL_MS * 1000ULL);
  }

  // Loss of response (no echo: no target, out of range or removed; this
  // cannot tell which). Stale at once, power and capture released, retried
  // with a fresh power-up on the saved mapping, then cleared.
  sonar.respond = false;
  CHECK(runUntil([] { return probeStatus().consecutiveFailures == 1; }, 1000));
  CHECK(phase() == ProbePhase::Retrying && probeStatus().readingStale && probeStatus().readingValid);
  CHECK(same(probeStatus().telemetry1, "Dist: 50.0 cm"));  // kept, marked stale
  CHECK(same(probeStatus().lastError, "no-echo"));
  CHECK(!backend.connected && allProbesPlainInput() && !anyProbeIsrAttached());
  const int connects = backend.connects;
  CHECK(runUntil([] { return phase() == ProbePhase::Idle; }, 10000));
  CHECK(!probeStatus().hasMapping && probeStatus().componentName == nullptr && !probeStatus().readingValid);
  CHECK(probeStatus().signalLabel[0] == nullptr && probeStatus().signalLabel[1] == nullptr);
  CHECK(backend.connects == connects + PROBE_MAX_CONSECUTIVE_FAILURES - 1);
  CHECK(!backend.connected && backend.violations == 0 && backend.signalsDrivenAtOff == 0);
  CHECK(allProbesPlainInput() && !anyProbeIsrAttached());

  // The response returns: rediscovered after the rescan interval.
  sonar.respond = true;
  CHECK(runUntil([] { return phase() == ProbePhase::Detected; }, PROBE_RESCAN_INTERVAL_MS + 3000));
  CHECK(backend.violations == 0);
}

// Every four-wire mapping is powered only through the backend, after a
// full break, and torn down (GPIOs, then supply and routes) before the
// next; the sonar answers only on its own mapping, even when an earlier
// one powers it correctly with TRIG and ECHO swapped.
static void testFourWireDiscoveryOrder() {
  FakeDriver f(1, "Fake F", ProbeTopology::FourWirePowered, ExcitationPolicy::ProtectedActive);
  useDrivers(&f);
  resetWorld(1000000ULL, kAllBackendCaps);
  CHECK(runUntil([] { return phase() == ProbePhase::NoComponent; }, 30000));
  CHECK(attemptCount == PROBE_FOUR_WIRE_COUNT);
  for (int i = 0; i < attemptCount; i++) CHECK(attempts[i].slot == i);
  CHECK(f.unpoweredStarts == 0 && f.unsafeResets == 0);
  CHECK(backend.connects == PROBE_FOUR_WIRE_COUNT && backend.violations == 0 && !backend.connected);
  CHECK(collectSessions().count == 0 && allProbesPlainInput());  // no GPIO ever drove a supply

  useDrivers(&ultrasonic);
  resetWorld(1000000ULL, kAllBackendCaps);
  sonar.attached = true;
  sonar.wiring = 16;  // C VCC, D TRIG, A ECHO, B GND; entry 13 (C, A, D, B) swaps TRIG/ECHO
  CHECK(runUntil([] { return phase() == ProbePhase::Detected; }, 20000));
  CHECK(sameRoles(probeStatus().mapping, PROBE_FOUR_WIRE_MAPPINGS[16]));
  CHECK(backend.connects == 17 && backend.violations == 0 && backend.signalsDrivenAtOff == 0);
  CHECK(sonar.triggers == ULTRASONIC_CONFIRM_SAMPLES + 1);
  char first[PROBE_MAPPING_TEXT_LEN];
  char second[PROBE_MAPPING_TEXT_LEN];
  formatProbeMapping(probeStatus().mapping, probeStatus().signalLabel, first, second, sizeof first);
  CHECK(same(first, "A:ECH B:GND") && same(second, "C:VCC D:TRG"));

  // Clearing tears the active mapping down: supply off, capture released,
  // no output left (the rescan's precheck pulls are already on).
  CHECK(probeRequestClear());
  CHECK(!backend.connected && !anyProbeIsrAttached());
  for (uint8_t g : PROBE_NODE_GPIO) CHECK(!mock::isOutput(g));
}

// A backend that declines to route never reaches the driver, and the scan
// moves on.
static void testBackendRefusal() {
  FakeDriver f(1, "Fake F", ProbeTopology::FourWirePowered, ExcitationPolicy::ProtectedActive);
  f.acceptSlot = 0;
  useDrivers(&f);
  resetWorld(1000000ULL, kAllBackendCaps);
  backend.refuse = true;
  CHECK(runUntil([] { return phase() == ProbePhase::NoComponent; }, 30000));
  CHECK(f.calls == 0);
  CHECK(backend.requests == PROBE_FOUR_WIRE_COUNT && backend.connects == 0 && backend.violations == 0);
  CHECK(same(probeStatus().lastError, "backend-refused") && allProbesPlainInput());
}

#endif  // PINDEX_POWERED_PROBING_ENABLED

#if FULL_BUILD

// ---- Topology routing with both kinds of driver ----------------------------

static void testTopologyEnumeration() {
  FakeDriver t(2, "Fake T", ProbeTopology::TwoTerminal, ExcitationPolicy::WeakPullOnly);
  FakeDriver u(3, "Fake U", ProbeTopology::TwoTerminal, ExcitationPolicy::ProtectedActive);  // unsupported
  FakeDriver p(1, "Fake P");
  FakeDriver f(4, "Fake F", ProbeTopology::FourWirePowered, ExcitationPolicy::ProtectedActive);
  useDrivers(&t, &u, &f, &p);  // registered passive-first on purpose
  resetWorld(1000000ULL, kAllBackendCaps);
  CHECK(runUntil([] { return phase() == ProbePhase::Scanning; }, 100));
  const int total = PROBE_PERMUTATION_COUNT + PROBE_FOUR_WIRE_COUNT + PROBE_PAIR_COUNT;
  CHECK(probeStatus().scanTotal == total);
  CHECK(runUntil([] { return phase() == ProbePhase::NoComponent; }, 60000));
  CHECK(attemptCount == total);
  for (int i = 0; i < PROBE_PERMUTATION_COUNT && i < attemptCount; i++) {  // three-wire first
    CHECK(attempts[i].driver == 1 && attempts[i].slot == i &&
          attempts[i].topology == ProbeTopology::ThreeWirePowered);
  }
  for (int i = 0; i < PROBE_FOUR_WIRE_COUNT; i++) {  // then four-wire, all 24 in order
    const Attempt &a = attempts[PROBE_PERMUTATION_COUNT + i];
    CHECK(a.driver == 4 && a.slot == i && a.topology == ProbeTopology::FourWirePowered);
  }
  for (int i = 0; i < PROBE_PAIR_COUNT; i++) {  // then each pair once: A-B, B-C, A-C, A-D, B-D, C-D
    const Attempt &a = attempts[PROBE_PERMUTATION_COUNT + PROBE_FOUR_WIRE_COUNT + i];
    CHECK(a.driver == 2 && a.slot == i && a.topology == ProbeTopology::TwoTerminal);
  }
  CHECK(u.calls == 0);                // unsupported capabilities are never attempted
  CHECK(t.energisedStarts == 0);      // weak-pull attempts start with every node plain INPUT
  CHECK(p.unpoweredStarts == 0 && f.unpoweredStarts == 0);
  CHECK(poweredSessions(collectSessions()) == PROBE_PERMUTATION_COUNT);  // GPIO power: three-wire only
  CHECK(backend.connects == PROBE_FOUR_WIRE_COUNT && backend.violations == 0 && !backend.connected);

  // An active protocol that answers ends the cycle before any passive
  // attempt (three-wire, then four-wire).
  FakeDriver t2(2, "Fake T", ProbeTopology::TwoTerminal, ExcitationPolicy::WeakPullOnly);
  FakeDriver p2(1, "Fake P");
  p2.acceptSlot = 2;
  useDrivers(&t2, &p2);
  resetWorld();
  CHECK(runUntil([] { return phase() == ProbePhase::Confirming; }, 10000));
  CHECK(t2.calls == 0 && same(probeStatus().componentName, "Fake P"));

  FakeDriver t3(2, "Fake T", ProbeTopology::TwoTerminal, ExcitationPolicy::WeakPullOnly);
  FakeDriver p3(1, "Fake P");
  FakeDriver f3(4, "Fake F", ProbeTopology::FourWirePowered, ExcitationPolicy::ProtectedActive);
  f3.acceptSlot = 5;
  useDrivers(&t3, &f3, &p3);
  resetWorld(1000000ULL, kAllBackendCaps);
  CHECK(runUntil([] { return phase() == ProbePhase::Detected; }, 40000));
  CHECK(t3.calls == 0 && p3.probeStarts == PROBE_PERMUTATION_COUNT && f3.probeStarts == 6);
  CHECK(same(probeStatus().componentName, "Fake F") && sameRoles(probeStatus().mapping, PROBE_FOUR_WIRE_MAPPINGS[5]));
}

// A contact (or a finite 1 kOhm path) between A and B: routed to passive
// measurement, and no powered attempt ever puts VCC on A or B.
static void testCouplingRoutesToPassive() {
  for (double ohms : {1.0, 1000.0}) {
    FakeDriver p(1, "Fake P");  // powered, never answers
    useDrivers(&p, &passive);
    attemptCount = 0;
    resetWorld();
    connect(0, 1, ohms);
    CHECK(runUntil([] { return phase() == ProbePhase::Detected; }, 30000));
    CHECK(probeStatus().precheck == PrecheckResult::CouplingObserved);
    CHECK((probeStatus().coupledPairs & probePairBit(0, 1)) != 0);
    CHECK(same(probeStatus().componentName, "Contact / Low R"));
    CHECK(same(probeStatus().telemetry1, "State: CLOSED"));
    CHECK(p.probeStarts == 12);  // VCC on C or D only
    for (int i = 0; i < attemptCount && !attempts[i].sample; i++) {
      if (attempts[i].topology != ProbeTopology::ThreeWirePowered) continue;
      CHECK(probeNodeWithRole(PROBE_PERMUTATIONS[attempts[i].slot], NodeRole::Vcc) >= 2);
    }
    const SessionLog log = collectSessions();
    CHECK(log.violations == 0 && poweredSessions(log) == 12);
    for (int i = 0; i < log.count; i++) CHECK(log.s[i].vcc < 0 || log.s[i].vcc == 2 || log.s[i].vcc == 3);

    // A constant wire stays a contact; passive monitoring powers nothing.
    runMs(5000);
    CHECK(same(probeStatus().componentName, "Contact / Low R"));
    CHECK(poweredSessions(collectSessions()) == 12);
  }
}

static void testDhtFirstThenPassive() {
  useDrivers(&dht, &ultrasonic, &passive);
  resetWorld(1000000ULL, kAllBackendCaps);
  sensor.attached = true;
  sensor.wiring = PROBE_PERMUTATIONS[0];
  CHECK(runUntil([] { return phase() == ProbePhase::Detected; }, 5000));
  CHECK(same(probeStatus().componentName, "DHT11 Temp & Hum"));
  CHECK(senseCount == 0);  // the passive fallback never ran
  CHECK(mock::attachCalls == 0 && backend.requests == 0);  // nor the lower-priority ultrasonic
  CHECK(adcReadsInCritical == 0);
}

// The shipped situation: no protected four-wire backend. The ultrasonic
// driver is registered but never energised or called and is reported as
// "hardware required"; DHT11 (here on a mapping that uses D) and the
// passive pairs run as before. Partial capability sets count as none.
static void testUltrasonicHardwareRequired() {
  useDrivers(&dht, &ultrasonic, &passive);
  resetWorld();  // backend capabilities: none
  CHECK(same(probeStatus().hardwareRequired, "HC-SR04 Ultrasonic"));
  sensor.attached = true;
  sensor.wiring = PROBE_PERMUTATIONS[8];  // B = VCC, A = DATA, D = GND
  CHECK(runUntil([] { return phase() == ProbePhase::Scanning; }, 100));
  CHECK(probeStatus().scanTotal == PROBE_PERMUTATION_COUNT + PROBE_PAIR_COUNT);  // no four-wire slots
  CHECK(runUntil([] { return phase() == ProbePhase::Detected; }, 40000));
  CHECK(same(probeStatus().componentName, "DHT11 Temp & Hum"));
  CHECK(same(probeStatus().hardwareRequired, "HC-SR04 Ultrasonic"));  // reported, not a fault
  CHECK(mock::attachCalls == 0 && backend.requests == 0 && highDriveEvents > 0);
  const SessionLog log = collectSessions();
  CHECK(log.violations == 0 && log.count == 9);
  for (int i = 0; i < log.count && i < 9; i++) CHECK(unusedStayedInput(log.s[i], PROBE_PERMUTATIONS[i]));

  // Passive still works after the full (four-wire-free) cycle.
  sensor.attached = false;
  connect(2, 3, 10000);
  CHECK(probeRequestClear());
  CHECK(runUntil([] { return phase() == ProbePhase::Detected; }, 60000));
  CHECK(same(probeStatus().componentName, "Resistive Component"));
  CHECK(mock::attachCalls == 0 && backend.requests == 0);

  const uint16_t partial[] = {kBackendFourWireBaseline,  // no 5 V supply
                              static_cast<uint16_t>(kAllBackendCaps & ~kBackendBreakBeforeMake),
                              static_cast<uint16_t>(kAllBackendCaps & ~kBackendSignalLevels),
                              static_cast<uint16_t>(kAllBackendCaps & ~kBackendAssignmentProtection)};
  for (uint16_t caps : partial) {
    useDrivers(&dht, &ultrasonic, &passive);
    resetWorld(1000000ULL, caps);
    runMs(3000);
    CHECK(probeStatus().scanTotal == PROBE_PERMUTATION_COUNT + PROBE_PAIR_COUNT);
    CHECK(same(probeStatus().hardwareRequired, "HC-SR04 Ultrasonic"));
    CHECK(mock::attachCalls == 0 && backend.requests == 0);
  }
}

// Any observed coupling vetoes every four-wire mapping (each drives or is
// driven on three of its four nodes); the precheck sees every pair, D
// included, and passive measurement still runs. Seeing no coupling grants
// nothing by itself.
static void testFourWirePrecheckVeto() {
  for (int i = 0; i < PROBE_PAIR_COUNT; i++) {
    FakeDriver f(1, "Fake F", ProbeTopology::FourWirePowered, ExcitationPolicy::ProtectedActive);
    FakeDriver t(2, "Fake T", ProbeTopology::TwoTerminal, ExcitationPolicy::WeakPullOnly);
    useDrivers(&f, &t);
    resetWorld(1000000ULL, kAllBackendCaps);
    const int x = probeNodeWithRole(PROBE_PAIRS[i], NodeRole::Terminal, 0);
    const int y = probeNodeWithRole(PROBE_PAIRS[i], NodeRole::Terminal, 1);
    connect(x, y, 1.0);
    CHECK(runUntil([] { return phase() == ProbePhase::NoComponent; }, 10000));
    CHECK(probeStatus().precheck == PrecheckResult::CouplingObserved);
    CHECK(probeStatus().coupledPairs == probePairBit(static_cast<uint8_t>(x), static_cast<uint8_t>(y)));
    CHECK(f.calls == 0 && backend.requests == 0);  // never powered
    CHECK(t.probeStarts == PROBE_PAIR_COUNT);
  }

  FakeDriver f(1, "Fake F", ProbeTopology::FourWirePowered, ExcitationPolicy::ProtectedActive);
  useDrivers(&f);
  resetWorld();  // an empty socket, but no capabilities
  runMs(10000);
  CHECK(f.calls == 0 && backend.requests == 0 && probeStatus().precheck == PrecheckResult::NotRun);
}

#endif  // FULL_BUILD

#if PINDEX_PASSIVE_PROBING_ENABLED

// ---- Passive driver through the engine --------------------------------------

static void testPassiveResistor() {
  useDrivers(&passive);
  resetWorld();
  connect(0, 1, 10000);
  CHECK(runUntil([] { return phase() == ProbePhase::Detected; }, 5000));
  CHECK(probeStatus().precheck == PrecheckResult::NoObviousCoupling);
  CHECK(probeStatus().scanTotal == PROBE_PAIR_COUNT);
  CHECK(same(probeStatus().componentName, "Resistive Component"));
  CHECK(same(probeStatus().telemetry1, "R~ 10.0 kOhm"));
  CHECK(same(probeStatus().telemetry2, "Signal: 81%"));
  char first[PROBE_MAPPING_TEXT_LEN];
  char second[PROBE_MAPPING_TEXT_LEN];
  formatProbeMapping(probeStatus().mapping, probeStatus().signalLabel, first, second, sizeof first);
  CHECK(same(first, "Pins: A & B") && same(second, "C:--- D:---"));

  // The first cycle visited each of the six pairs once, both orientations,
  // in order (a reversed orientation is not another pair).
  const uint8_t a = gpioOf(0), b = gpioOf(1), c = gpioOf(2), d = gpioOf(3);
  const uint8_t expected[2 * PROBE_PAIR_COUNT] = {a, b, b, c, a, c, a, d, b, d, c, d};
  CHECK(senseCount >= 2 * PROBE_PAIR_COUNT);
  for (int i = 0; i < 2 * PROBE_PAIR_COUNT; i++) CHECK(senseLog[i] == expected[i]);

  // A fixed resistor never becomes "variable" or an LDR on its own, and
  // nothing is ever driven HIGH or push-pull.
  runMs(10000);
  CHECK(phase() == ProbePhase::Detected && probeStatus().consecutiveFailures == 0);
  CHECK(same(probeStatus().componentName, "Resistive Component"));
  CHECK(highDriveEvents == 0);
  bool pushPull = false;
  for (size_t i = 0; i < mock::eventCount; i++) {
    if (nodeOf(mock::events[i].pin) >= 0 && mock::events[i].mode == OUTPUT) pushPull = true;
  }
  CHECK(!pushPull);
}

// A resistor across C-D: the pair on GPIO33 (ADC1 on the ESP32; the
// simulated adapter here is unit-agnostic) is found and labelled.
static void testPassiveOnNodeD() {
  useDrivers(&passive);
  resetWorld();
  connect(2, 3, 10000);
  CHECK(runUntil([] { return phase() == ProbePhase::Detected; }, 5000));
  CHECK(same(probeStatus().telemetry1, "R~ 10.0 kOhm"));
  CHECK(sameRoles(probeStatus().mapping, PROBE_PAIRS[5]));
  char first[PROBE_MAPPING_TEXT_LEN];
  char second[PROBE_MAPPING_TEXT_LEN];
  formatProbeMapping(probeStatus().mapping, probeStatus().signalLabel, first, second, sizeof first);
  CHECK(same(first, "Pins: C & D") && same(second, "A:--- B:---"));
  CHECK(highDriveEvents == 0);
}

static void testSwitchBehaviour() {
  useDrivers(&passive);
  resetWorld();
  connect(0, 1, 1.0);
  CHECK(runUntil([] { return phase() == ProbePhase::Detected; }, 5000));
  CHECK(same(probeStatus().componentName, "Contact / Low R"));
  CHECK(same(probeStatus().telemetry1, "State: CLOSED"));
  CHECK(!probeSendCommand('l'));  // guided LDR mode does not apply to contacts
  CHECK(waitSamples(20));
  CHECK(same(probeStatus().componentName, "Contact / Low R"));  // constant: no button claim

  // A single open sample is debounced away.
  connect(0, 1, kOpenOhms);
  CHECK(waitSamples(1));
  connect(0, 1, 1.0);
  CHECK(waitSamples(3));
  CHECK(same(probeStatus().componentName, "Contact / Low R"));
  CHECK(same(probeStatus().telemetry1, "State: CLOSED"));

  // A sustained opening is switch-like; OPEN is valid telemetry, not a failure.
  connect(0, 1, kOpenOhms);
  CHECK(waitSamples(3));
  CHECK(same(probeStatus().componentName, "Push Button / Switch"));
  CHECK(same(probeStatus().telemetry1, "State: OPEN") && probeStatus().telemetry2[0] == '\0');
  CHECK(phase() == ProbePhase::Detected && !probeStatus().readingStale);
  runMs(10000);  // an open switch is never treated as removal
  CHECK(phase() == ProbePhase::Detected && probeStatus().consecutiveFailures == 0);
  CHECK(same(probeStatus().telemetry1, "State: OPEN"));
  connect(0, 1, 1.0);
  CHECK(waitSamples(3));
  CHECK(same(probeStatus().telemetry1, "State: CLOSED"));

  // Clearing forgets it; an open socket is never identified.
  connect(0, 1, kOpenOhms);
  CHECK(probeRequestClear());
  CHECK(runUntil([] { return phase() == ProbePhase::NoComponent; }, 5000));
  CHECK(!probeStatus().hasMapping && probeStatus().componentName == nullptr);
  CHECK(!probeStatus().measurementUnavailable);
}

static void testAmbiguousCoupling() {
  useDrivers(&passive);
  resetWorld();
  connect(0, 1, 10000);
  connect(1, 2, 10000);  // a three-node network: every pair conducts
  CHECK(runUntil([] { return phase() == ProbePhase::NoComponent; }, 5000));
  CHECK(same(probeStatus().lastError, "ambiguous-pairs"));
  CHECK(!probeStatus().hasMapping);
}

static void testVariableAndGuidedLdr() {
  useDrivers(&passive);
  resetWorld();
  connect(0, 1, 10000);
  CHECK(runUntil([] { return phase() == ProbePhase::Detected; }, 5000));
  CHECK(!probeSendCommand('d'));  // no guided step pending

  // A sustained change above the filtered noise: variable, possibly an LDR.
  connect(0, 1, 30000);
  CHECK(waitSamples(12));
  CHECK(same(probeStatus().componentName, "Variable Resistance"));
  CHECK(same(probeStatus().telemetry1, "R~ 30.0 kOhm"));
  CHECK(strncmp(probeStatus().telemetry2, "Sig ", 4) == 0);
  CHECK(strstr(probeStatus().telemetry2, "Possible LDR") != nullptr);

  // Guided calibration: cover (dark), then light (bright).
  CHECK(probeSendCommand('l'));
  CHECK(waitSamples(1));
  CHECK(same(probeStatus().telemetry1, "LDR cal: cover it"));
  CHECK(same(probeStatus().telemetry2, "then send 'd'"));
  connect(0, 1, 100000);  // dark: 2276 mV
  CHECK(waitSamples(2));
  CHECK(probeSendCommand('d'));
  CHECK(waitSamples(PASSIVE_LDR_WINDOW_SAMPLES + 1));
  CHECK(same(probeStatus().telemetry1, "LDR cal: light it"));
  connect(0, 1, 5000);  // bright: 330 mV
  CHECK(waitSamples(2));
  CHECK(probeSendCommand('b'));
  CHECK(waitSamples(PASSIVE_LDR_WINDOW_SAMPLES + 1));
  CHECK(same(probeStatus().componentName, "Photoresistor (LDR)"));
  CHECK(same(probeStatus().telemetry1, "Light: 100%"));
  CHECK(same(probeStatus().telemetry2, "R~ 5.0 kOhm"));

  // The index is relative to those references and clamped to 0-100.
  connect(0, 1, 20000);  // 1015 mV -> (2276 - 1015) * 100 / 1946
  CHECK(waitSamples(2));
  CHECK(same(probeStatus().telemetry1, "Light: 64%"));
  connect(0, 1, 1000);  // brighter than the bright reference
  CHECK(waitSamples(2));
  CHECK(same(probeStatus().telemetry1, "Light: 100%") && same(probeStatus().telemetry2, "R~ < 2.1 kOhm"));
  connect(0, 1, 120000);  // darker than the dark reference
  CHECK(waitSamples(2));
  CHECK(same(probeStatus().telemetry1, "Light: 0%"));

  // Calibration belongs to the session: clearing invalidates it.
  CHECK(probeRequestClear());
  CHECK(runUntil([] { return phase() == ProbePhase::Detected; }, 5000));
  CHECK(same(probeStatus().componentName, "Resistive Component"));

  // Too little variation: rejected, no percentage invented.
  connect(0, 1, 10000);
  CHECK(probeRequestClear());
  CHECK(runUntil([] { return phase() == ProbePhase::Detected; }, 5000));
  CHECK(probeSendCommand('l'));
  CHECK(waitSamples(2));
  CHECK(probeSendCommand('d'));
  CHECK(waitSamples(PASSIVE_LDR_WINDOW_SAMPLES + 1));
  connect(0, 1, 11000);
  CHECK(waitSamples(2));
  CHECK(probeSendCommand('b'));
  CHECK(waitSamples(PASSIVE_LDR_WINDOW_SAMPLES + 1));
  CHECK(same(probeStatus().componentName, "Resistive Component"));
  CHECK(same(probeStatus().telemetry2, "Cal failed: no change"));
  CHECK(strncmp(probeStatus().telemetry1, "Light", 5) != 0);
}

static void testResistiveOpenGrace() {
  useDrivers(&passive);
  resetWorld();
  connect(0, 1, 10000);
  CHECK(runUntil([] { return phase() == ProbePhase::Detected; }, 5000));
  connect(0, 1, kOpenOhms);  // removed, or a very high resistance: unknowable here
  CHECK(waitSamples(3));
  CHECK(phase() == ProbePhase::Detected && !probeStatus().readingStale);
  CHECK(same(probeStatus().telemetry1, "R~ > 130 kOhm"));
  CHECK(same(probeStatus().telemetry2, "Open or removed?"));
  CHECK(runUntil([] { return probeStatus().consecutiveFailures == 1; },
                 (PASSIVE_OPEN_GRACE_SAMPLES + 5) * PASSIVE_SAMPLE_PERIOD_MS));
  CHECK(runUntil([] { return phase() == ProbePhase::Idle; }, 10000));
  CHECK(!probeStatus().hasMapping && allProbesPlainInput());
}

static void testMeasurementUnavailable() {
  useDrivers(&passive);
  resetWorld();
  connect(0, 1, 10000);
  net.adcUnavailable = true;
  CHECK(runUntil([] { return phase() == ProbePhase::NoComponent; }, 5000));
  CHECK(probeStatus().measurementUnavailable);
  CHECK(same(probeStatus().lastError, "adc-unavailable"));
  CHECK(allProbesPlainInput());
  for (size_t i = 0; i < mock::eventCount; i++) {  // the sink is never engaged
    CHECK(!(mock::events[i].isPinMode && mock::events[i].mode == OUTPUT_OPEN_DRAIN));
  }

  resetWorld();  // a failing conversion is an error, never a 0 mV contact
  net.adcReadFails = true;
  CHECK(runUntil([] { return phase() == ProbePhase::NoComponent; }, 5000));
  CHECK(probeStatus().measurementUnavailable && same(probeStatus().lastError, "adc-read-failed"));

  resetWorld();  // an empty socket is "no component", not a measurement problem
  CHECK(runUntil([] { return phase() == ProbePhase::NoComponent; }, 5000));
  CHECK(!probeStatus().measurementUnavailable);
}

#endif  // PINDEX_PASSIVE_PROBING_ENABLED

#if PASSIVE_ONLY_BUILD

// Powered probing off, passive on: the DHT and ultrasonic drivers are
// registered but never run (even with a capable backend), nothing is
// powered or driven HIGH or push-pull, and passive detection works.
static void testPassiveOnlyBuild() {
  useDrivers(&dht, &ultrasonic, &passive);
  resetWorld(1000000ULL, kAllBackendCaps);
  connect(0, 1, 10000);
  CHECK(runUntil([] { return phase() == ProbePhase::Scanning; }, 100));
  CHECK(probeStatus().scanTotal == PROBE_PAIR_COUNT);
  CHECK(probeStatus().hardwareRequired == nullptr);  // disabled, not missing hardware
  CHECK(runUntil([] { return phase() == ProbePhase::Detected; }, 5000));
  CHECK(same(probeStatus().componentName, "Resistive Component"));
  CHECK(highDriveEvents == 0 && backend.requests == 0 && mock::attachCalls == 0);
  for (size_t i = 0; i < mock::eventCount; i++) {
    if (nodeOf(mock::events[i].pin) < 0) continue;
    CHECK(mock::events[i].mode != OUTPUT && mock::events[i].mode != (OUTPUT_OPEN_DRAIN | PULLUP));
  }
}

#endif

#if !PINDEX_POWERED_PROBING_ENABLED && !PINDEX_PASSIVE_PROBING_ENABLED

static void testDisabledNeverDrives() {
  FakeDriver a(1, "Fake A");
  FakeDriver t(2, "Fake T", ProbeTopology::TwoTerminal, ExcitationPolicy::WeakPullOnly);
  a.acceptSlot = 0;
  t.acceptSlot = 0;
  useDrivers(&a, &t, &ultrasonic, &passive);
  resetWorld(1000000ULL, kAllBackendCaps);  // not even a capable backend is used
  connect(0, 1, 10000);
  const uint32_t rev = probeStatus().revision;
  runMs(30000);
  CHECK(phase() == ProbePhase::Disabled);
  CHECK(probeStatus().revision == rev && probeStatus().hardwareRequired == nullptr);
  CHECK(!probeRequestRetry() && !probeRequestClear());  // never bypasses the setting
  runMs(1000);
  CHECK(phase() == ProbePhase::Disabled);
  CHECK(a.calls == 0 && t.calls == 0 && attemptCount == 0 && senseCount == 0);
  CHECK(backend.requests == 0 && mock::attachCalls == 0);

  int probeEvents = 0;
  for (size_t i = 0; i < mock::eventCount; i++) {
    const mock::Event &e = mock::events[i];
    if (nodeOf(e.pin) < 0) continue;
    probeEvents++;
    CHECK(e.isPinMode && e.mode == INPUT);  // no output, no pulls, no writes
  }
  CHECK(probeEvents == PROBE_NODE_COUNT);  // only the initial resetTestPins()
  CHECK(allProbesPlainInput());
}

#endif

int main() {
  netReset();
  testFrameDecoding();
  testPulseDecoding();
  testMappingTables();
  testMappingText();
  testPassiveEstimate();
  testUltrasonicEchoMath();
  testDht11DriverContract();
  testPassiveDriverContract();
  testUltrasonicDriverContract();
  testPassiveToDhtHandoff();
  testUltrasonicHandoff();
#if PINDEX_POWERED_PROBING_ENABLED
  testEmptySocketScan();
  testDetectionNeedsTwoFrames();
  testDhtOnNodeD();
  testStaleThenRecovery();
  testRemovalAfterThreeFailures();
  testCaptureErrors();
  testPrecheckRouting();
  testRollover();
  testPermutationMajorDriverOrder();
  testLaterDriverAndConfirmation();
  testPendingIsNotFailure();
  testRequirementsDriveScheduling();
  testRemovalClearsActiveDriver();
  testEmptyRegistry();
  testUltrasonicDetection();
  testFourWireDiscoveryOrder();
  testBackendRefusal();
#endif
#if FULL_BUILD
  testTopologyEnumeration();
  testCouplingRoutesToPassive();
  testDhtFirstThenPassive();
  testUltrasonicHardwareRequired();
  testFourWirePrecheckVeto();
#endif
#if PINDEX_PASSIVE_PROBING_ENABLED
  testPassiveResistor();
  testPassiveOnNodeD();
  testSwitchBehaviour();
  testAmbiguousCoupling();
  testVariableAndGuidedLdr();
  testResistiveOpenGrace();
  testMeasurementUnavailable();
#endif
#if PASSIVE_ONLY_BUILD
  testPassiveOnlyBuild();
#endif
#if !PINDEX_POWERED_PROBING_ENABLED && !PINDEX_PASSIVE_PROBING_ENABLED
  testDisabledNeverDrives();
#endif
  CHECK(lockLeaks == 0);           // the capture lock was always released
  CHECK(lockedPinModes == 0);      // pinMode() never ran inside the capture lock
  CHECK(adcReadsInCritical == 0);  // no ADC work inside the DHT critical section
  CHECK(adcWithIsrAttached == 0);  // ultrasonic interrupts released before any ADC use
  printf("build powered=%d passive=%d: %d checks, %d failed\n", PINDEX_POWERED_PROBING_ENABLED,
         PINDEX_PASSIVE_PROBING_ENABLED, checks, failures);
  return failures == 0 ? 0 : 1;
}
