// Host tests for PinDex. Not part of the Arduino build: the IDE compiles
// only the sketch folder root and src/, never tests/.
// Build from the PinDex folder with any C++17 host compiler, three ways:
//   SRC="tests/test_pindex.cpp probe_engine.cpp dht11.cpp
//        src/drivers/dht11_driver.cpp src/drivers/passive_driver.cpp"
//   g++ -std=c++17 -Wall -Wextra -Itests/mock -I. -DPINDEX_POWERED_PROBING_ENABLED=1
//       -DPINDEX_PASSIVE_PROBING_ENABLED=1 $SRC -o build/test_all
//   ...=0 / ...PASSIVE...=1 -> build/test_passive_only
//   ...=0 / ...PASSIVE...=0 -> build/test_disabled
// Run all three; each exits non-zero if any check fails. This file supplies
// the driver registry and the passive ADC adapter itself (link seams), so it
// is linked WITHOUT src/drivers/driver_registry.cpp and passive_adc.cpp.
// Shorts, resistors, faults and sensors are simulated; nothing here touches
// hardware, and no test drives opposing GPIO outputs into each other.

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <initializer_list>

#include <Arduino.h>  // tests/mock/Arduino.h

#include "config.h"
#include "dht11.h"
#include "probe_engine.h"
#include "sensor_driver.h"
#include "src/drivers/dht11_driver.h"
#include "src/drivers/driver_registry.h"
#include "src/drivers/passive_adc.h"
#include "src/drivers/passive_driver.h"

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
                       SensorDriver *c = nullptr) {
  testDriverCount = 0;
  for (SensorDriver *d : {a, b, c}) {
    if (d != nullptr) testDrivers[testDriverCount++] = d;
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

static void testMappingText() {
  // {VCC, DATA, GND} node indices of the fixed permutation order.
  const int order[PROBE_PERMUTATION_COUNT][3] = {
      {0, 1, 2}, {2, 1, 0}, {1, 0, 2}, {1, 2, 0}, {0, 2, 1}, {2, 0, 1}};
  const char *text[PROBE_PERMUTATION_COUNT] = {
      "A:VCC | B:DAT | C:GND", "A:GND | B:DAT | C:VCC", "A:DAT | B:VCC | C:GND",
      "A:GND | B:VCC | C:DAT", "A:VCC | B:GND | C:DAT", "A:DAT | B:GND | C:VCC"};
  char buf[PROBE_MAPPING_TEXT_LEN];
  for (int i = 0; i < PROBE_PERMUTATION_COUNT; i++) {
    const ProbeMapping &m = PROBE_PERMUTATIONS[i];
    CHECK(m.topology == ProbeTopology::ThreeWirePowered);
    CHECK(probeNodeWithRole(m, NodeRole::Vcc) == order[i][0] &&
          probeNodeWithRole(m, NodeRole::Data) == order[i][1] &&
          probeNodeWithRole(m, NodeRole::Gnd) == order[i][2]);
    formatProbeMapping(m, buf, sizeof buf);
    CHECK(strcmp(buf, text[i]) == 0);
    CHECK(strlen(buf) * 6 <= OLED_WIDTH);  // size-1 font is 6 px per glyph
  }

  const char *pairText[PROBE_PAIR_COUNT] = {"Pins: A & B, C:---", "Pins: B & C, A:---",
                                            "Pins: A & C, B:---"};
  for (int i = 0; i < PROBE_PAIR_COUNT; i++) {
    const ProbeMapping &m = PROBE_PAIRS[i];
    CHECK(m.topology == ProbeTopology::TwoTerminal);
    CHECK(probeNodeWithRole(m, NodeRole::Terminal, 1) >= 0 && probeNodeWithRole(m, NodeRole::Unused) >= 0);
    CHECK(probeNodeWithRole(m, NodeRole::Vcc) < 0 && probeNodeWithRole(m, NodeRole::Gnd) < 0);
    formatProbeMapping(m, buf, sizeof buf);
    CHECK(strcmp(buf, pairText[i]) == 0);
  }
  CHECK(probePairBit(0, 1) == 1 && probePairBit(1, 2) == 2 && probePairBit(0, 2) == 4);
  CHECK(probePairBit(1, 0) == 1 && probePairBit(1, 1) == 0);
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

static const uint8_t kProbeGpio[PROBE_NODE_COUNT] = {PROBE_PIN_A, PROBE_PIN_B, PROBE_PIN_C};
static int lockLeaks = 0;
static int lockedPinModes = 0;
static int highDriveEvents = 0;  // any probe pin driven HIGH (push-pull)
static int adcReadsInCritical = 0;
static bool flakyA = false;  // A alternates HIGH/LOW while pulled up
static int flakyReads = 0;

// Operation starts logged by FakeDriver.
struct Attempt {
  int driver;
  int slot;  // permutation index (three-wire) or pair index (two-terminal)
  bool sample;
  bool twoTerminal;
  uint64_t us;
};
static Attempt attempts[256];
static int attemptCount = 0;

static uint8_t gpioOf(int node) { return kProbeGpio[node]; }

static int nodeOf(uint8_t pin) {
  for (int n = 0; n < PROBE_NODE_COUNT; n++) {
    if (kProbeGpio[n] == pin) return n;
  }
  return -1;
}

static bool allProbesPlainInput() {
  for (uint8_t g : kProbeGpio) {
    if (mock::pins[g].mode != INPUT) return false;
  }
  return true;
}

static int permutationOf(const ThreeWirePins &p) {
  const int v = nodeOf(p.vcc), d = nodeOf(p.data), g = nodeOf(p.gnd);
  if (v < 0 || d < 0 || g < 0) return -1;
  for (int i = 0; i < PROBE_PERMUTATION_COUNT; i++) {
    const ProbeMapping &m = PROBE_PERMUTATIONS[i];
    if (m.role[v] == NodeRole::Vcc && m.role[d] == NodeRole::Data && m.role[g] == NodeRole::Gnd) {
      return i;
    }
  }
  return -1;
}

static int pairOf(const TwoTerminalPins &p) {
  const int a = nodeOf(p.first), b = nodeOf(p.second), u = nodeOf(p.unused);
  if (a < 0 || b < 0 || u < 0) return -1;
  for (int i = 0; i < PROBE_PAIR_COUNT; i++) {
    const ProbeMapping &m = PROBE_PAIRS[i];
    if (m.role[a] == NodeRole::Terminal && m.role[b] == NodeRole::Terminal &&
        m.role[u] == NodeRole::Unused) {
      return i;
    }
  }
  return -1;
}

[[maybe_unused]] static ProbeContext poweredContext(int perm) {
  const ProbeMapping &m = PROBE_PERMUTATIONS[perm];
  ProbeContext c{};
  c.topology = ProbeTopology::ThreeWirePowered;
  c.powered = {gpioOf(probeNodeWithRole(m, NodeRole::Vcc)), gpioOf(probeNodeWithRole(m, NodeRole::Data)),
               gpioOf(probeNodeWithRole(m, NodeRole::Gnd))};
  return c;
}

[[maybe_unused]] static ProbeContext pairContext(int pair) {
  const ProbeMapping &m = PROBE_PAIRS[pair];
  ProbeContext c{};
  c.topology = ProbeTopology::TwoTerminal;
  c.pair = {gpioOf(probeNodeWithRole(m, NodeRole::Terminal, 0)),
            gpioOf(probeNodeWithRole(m, NodeRole::Terminal, 1)),
            gpioOf(probeNodeWithRole(m, NodeRole::Unused))};
  return c;
}

[[maybe_unused]] static bool sameRoles(const ProbeMapping &a, const ProbeMapping &b) {
  return a.topology == b.topology && memcmp(a.role, b.role, sizeof a.role) == 0;
}

// One loop() iteration per simulated millisecond.
static void tick() {
  mock::nowUs += 1000;
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
    mock::nowUs += 1000;
  }
  return false;
}

[[maybe_unused]] static bool driveSample(SensorDriver &d, char *l1, char *l2, size_t len) {
  for (int i = 0; i < 1000; i++) {
    if (d.readSample(l1, l2, len)) return true;
    if (d.getOperationStatus() != DriverOperationStatus::Pending) return false;
    mock::nowUs += 1000;
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
    const mock::Pin &p = mock::pins[kProbeGpio[i]];
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
  for (uint8_t g : kProbeGpio) {
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

// Called after every pinMode()/digitalWrite().
static void onPinChange(uint8_t pin, bool isPinMode) {
  if (isPinMode && mock::criticalDepth != 0) lockedPinModes++;
  if (nodeOf(pin) >= 0 && mock::drivesHigh(pin)) highDriveEvents++;
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

// Call useDrivers() first: init resets every registered driver.
static void resetWorld(uint64_t startUs = 1000000ULL) {
  mock::reset(startUs);
  sensor = SimSensor();
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
      : req{topology, excitation, 300, 500, 700, 1000}, id_(id), name_(name) {}

  DriverRequirements req;
  int pendingCalls = 0;  // Pending results before each operation completes
  int acceptSlot = -1;   // probe() succeeds only on this permutation/pair index
  bool samplesSucceed = true;

  int probeStarts = 0;
  int sampleStarts = 0;
  int calls = 0;
  int resets = 0;
  int unsafeResets = 0;     // reset() while a probe pin was not plain INPUT
  int unpoweredStarts = 0;  // three-wire operation started without VCC/GND applied
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
      slot_ = context.topology == ProbeTopology::ThreeWirePowered ? permutationOf(context.powered)
                                                                  : pairOf(context.pair);
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
    } else if (!allProbesPlainInput()) {
      energisedStarts++;
    }
    if (attemptCount < 256) {
      attempts[attemptCount++] =
          Attempt{id_, slot_, sample, req.topology == ProbeTopology::TwoTerminal, mock::nowUs};
    }
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
// driven low (covers GND-first power-up and GND-last teardown), DATA not
// released before VCC, or DATA driven high push-pull (shows as two supplies).
[[maybe_unused]] static SessionLog collectSessions() {
  SessionLog log;
  uint8_t mode[PROBE_NODE_COUNT] = {0, 0, 0};
  uint8_t latch[PROBE_NODE_COUNT] = {0, 0, 0};
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
    if (highBefore >= 0 && highNode < 0 && cur.gnd >= 0) {
      const int dataNode = 3 - highBefore - cur.gnd;  // node indices sum to 0+1+2
      if (mode[dataNode] != INPUT) log.violations++;
    }
    if (active) {
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

  // Every pair, both orientations (sense first terminal, then second); the
  // unused node is never touched; the sink is open-drain only.
  for (int i = 0; i < PROBE_PAIR_COUNT; i++) {
    const ProbeMapping &m = PROBE_PAIRS[i];
    const int t0 = probeNodeWithRole(m, NodeRole::Terminal, 0);
    const int t1 = probeNodeWithRole(m, NodeRole::Terminal, 1);
    const int unused = probeNodeWithRole(m, NodeRole::Unused);
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
      if (ev.pin == gpioOf(unused)) unusedTouched = true;
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
  for (uint8_t g : kProbeGpio) CHECK(mock::pins[g].mode == INPUT);  // no analog/pull state left
  netReset();
  sensor.attached = true;
  sensor.wiring = PROBE_PERMUTATIONS[0];
  powerByHand(PROBE_PERMUTATIONS[0]);
  CHECK(driveProbe(dd, poweredContext(0)));
  CHECK(sensor.responses == 1);
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
  return runUntil([] { return phase() == ProbePhase::Detected; }, 20000);
}

static void testEmptySocketScan() {
  resetWithDht();
  const uint64_t t0 = mock::nowUs;
  CHECK(runUntil([] { return phase() == ProbePhase::NoComponent; }, 20000));
  CHECK(same(probeStatus().lastError, dhtErrorName(DhtError::NoResponse)));
  CHECK(probeStatus().precheck == PrecheckResult::NoObviousCoupling);
  CHECK(!probeStatus().hasMapping && !probeStatus().readingValid);
  CHECK(probeStatus().componentName == nullptr && !probeStatus().measurementUnavailable);
  CHECK(allProbesPlainInput());

  // Latency is dominated by per-permutation stabilization (~1.46 s each).
  const uint64_t scanMs = (mock::nowUs - t0) / 1000;
  CHECK(scanMs >= 6 * DHT_POWER_STABILIZE_MS && scanMs <= 6 * 1600);

  const SessionLog log = collectSessions();
  CHECK(log.violations == 0);
  CHECK(log.count == PROBE_PERMUTATION_COUNT);
  for (int i = 0; i < log.count && i < PROBE_PERMUTATION_COUNT; i++) {
    const Session &x = log.s[i];
    CHECK(sessionUses(x, PROBE_PERMUTATIONS[i]));  // exact table order
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
  char text[PROBE_MAPPING_TEXT_LEN];
  formatProbeMapping(probeStatus().mapping, text, sizeof text);
  CHECK(strcmp(text, "A:GND | B:VCC | C:DAT") == 0);

  // Permutations 1-3 were torn down; 4 stayed powered for both frames.
  SessionLog log = collectSessions();
  CHECK(log.violations == 0);
  CHECK(log.count == 4);
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
  CHECK(runUntil([] { return phase() == ProbePhase::NoComponent; }, 20000));
  CHECK(probeStatus().precheck == PrecheckResult::CouplingObserved);
  CHECK(probeStatus().coupledPairs == probePairBit(0, 1));
  SessionLog log = collectSessions();
  CHECK(log.violations == 0);
  CHECK(poweredSessions(log) == 2);  // only permutations 2 and 6 (VCC on C)
  for (int i = 0; i < log.count; i++) CHECK(log.s[i].vcc == 2);
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
  CHECK(runUntil([] { return phase() == ProbePhase::NoComponent; }, 60000));
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
    char m[PROBE_MAPPING_TEXT_LEN];
    formatProbeMapping(probeStatus().mapping, m, sizeof m);
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

#endif  // PINDEX_POWERED_PROBING_ENABLED

#if FULL_BUILD

// ---- Topology routing with both kinds of driver ----------------------------

static void testTopologyEnumeration() {
  FakeDriver t(2, "Fake T", ProbeTopology::TwoTerminal, ExcitationPolicy::WeakPullOnly);
  FakeDriver u(3, "Fake U", ProbeTopology::TwoTerminal, ExcitationPolicy::ProtectedActive);  // unsupported
  FakeDriver p(1, "Fake P");
  useDrivers(&t, &u, &p);  // registered passive-first on purpose
  attemptCount = 0;
  resetWorld();
  CHECK(runUntil([] { return phase() == ProbePhase::Scanning; }, 100));
  CHECK(probeStatus().scanTotal == PROBE_PERMUTATION_COUNT + PROBE_PAIR_COUNT);
  CHECK(runUntil([] { return phase() == ProbePhase::NoComponent; }, 30000));
  CHECK(attemptCount == PROBE_PERMUTATION_COUNT + PROBE_PAIR_COUNT);
  for (int i = 0; i < PROBE_PERMUTATION_COUNT && i < attemptCount; i++) {  // powered first
    CHECK(attempts[i].driver == 1 && attempts[i].slot == i && !attempts[i].twoTerminal);
  }
  for (int i = 0; i < PROBE_PAIR_COUNT; i++) {  // then each pair once: A-B, B-C, A-C
    const Attempt &a = attempts[PROBE_PERMUTATION_COUNT + i];
    CHECK(a.driver == 2 && a.slot == i && a.twoTerminal);
  }
  CHECK(u.calls == 0);                // unsupported capabilities are never attempted
  CHECK(t.energisedStarts == 0);      // weak-pull attempts start with every node plain INPUT
  CHECK(p.unpoweredStarts == 0);
  CHECK(poweredSessions(collectSessions()) == PROBE_PERMUTATION_COUNT);

  // An active protocol that answers ends the cycle before any passive attempt.
  FakeDriver t2(2, "Fake T", ProbeTopology::TwoTerminal, ExcitationPolicy::WeakPullOnly);
  FakeDriver p2(1, "Fake P");
  p2.acceptSlot = 2;
  useDrivers(&t2, &p2);
  resetWorld();
  CHECK(runUntil([] { return phase() == ProbePhase::Confirming; }, 10000));
  CHECK(t2.calls == 0 && same(probeStatus().componentName, "Fake P"));
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
    CHECK(runUntil([] { return phase() == ProbePhase::Detected; }, 15000));
    CHECK(probeStatus().precheck == PrecheckResult::CouplingObserved);
    CHECK((probeStatus().coupledPairs & probePairBit(0, 1)) != 0);
    CHECK(same(probeStatus().componentName, "Contact / Low R"));
    CHECK(same(probeStatus().telemetry1, "State: CLOSED"));
    CHECK(p.probeStarts == 2 && attempts[0].slot == 1 && attempts[1].slot == 5);  // VCC on C only
    const SessionLog log = collectSessions();
    CHECK(log.violations == 0 && poweredSessions(log) == 2);
    for (int i = 0; i < log.count; i++) CHECK(log.s[i].vcc < 0 || log.s[i].vcc == 2);

    // A constant wire stays a contact; passive monitoring powers nothing.
    runMs(5000);
    CHECK(same(probeStatus().componentName, "Contact / Low R"));
    CHECK(poweredSessions(collectSessions()) == 2);
  }
}

static void testDhtFirstThenPassive() {
  useDrivers(&dht, &passive);
  resetWorld();
  sensor.attached = true;
  sensor.wiring = PROBE_PERMUTATIONS[0];
  CHECK(runUntil([] { return phase() == ProbePhase::Detected; }, 5000));
  CHECK(same(probeStatus().componentName, "DHT11 Temp & Hum"));
  CHECK(senseCount == 0);  // the passive fallback never ran
  CHECK(adcReadsInCritical == 0);
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
  char text[PROBE_MAPPING_TEXT_LEN];
  formatProbeMapping(probeStatus().mapping, text, sizeof text);
  CHECK(same(text, "Pins: A & B, C:---"));

  // The first cycle visited each pair once, both orientations, in order.
  const uint8_t a = gpioOf(0), b = gpioOf(1), c = gpioOf(2);
  const uint8_t expected[6] = {a, b, b, c, a, c};
  CHECK(senseCount >= 6);
  for (int i = 0; i < 6; i++) CHECK(senseLog[i] == expected[i]);

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

// Powered probing off, passive on: the DHT driver is registered but never
// runs, nothing is driven HIGH or push-pull, and passive detection works.
static void testPassiveOnlyBuild() {
  useDrivers(&dht, &passive);
  resetWorld();
  connect(0, 1, 10000);
  CHECK(runUntil([] { return phase() == ProbePhase::Scanning; }, 100));
  CHECK(probeStatus().scanTotal == PROBE_PAIR_COUNT);
  CHECK(runUntil([] { return phase() == ProbePhase::Detected; }, 5000));
  CHECK(same(probeStatus().componentName, "Resistive Component"));
  CHECK(highDriveEvents == 0);
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
  useDrivers(&a, &t, &passive);
  resetWorld();
  connect(0, 1, 10000);
  const uint32_t rev = probeStatus().revision;
  runMs(30000);
  CHECK(phase() == ProbePhase::Disabled);
  CHECK(probeStatus().revision == rev);
  CHECK(!probeRequestRetry() && !probeRequestClear());  // never bypasses the setting
  runMs(1000);
  CHECK(phase() == ProbePhase::Disabled);
  CHECK(a.calls == 0 && t.calls == 0 && attemptCount == 0 && senseCount == 0);

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
  testMappingText();
  testPassiveEstimate();
  testDht11DriverContract();
  testPassiveDriverContract();
  testPassiveToDhtHandoff();
#if PINDEX_POWERED_PROBING_ENABLED
  testEmptySocketScan();
  testDetectionNeedsTwoFrames();
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
#endif
#if FULL_BUILD
  testTopologyEnumeration();
  testCouplingRoutesToPassive();
  testDhtFirstThenPassive();
#endif
#if PINDEX_PASSIVE_PROBING_ENABLED
  testPassiveResistor();
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
  printf("build powered=%d passive=%d: %d checks, %d failed\n", PINDEX_POWERED_PROBING_ENABLED,
         PINDEX_PASSIVE_PROBING_ENABLED, checks, failures);
  return failures == 0 ? 0 : 1;
}
