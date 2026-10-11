// Host tests for PinDex. Not part of the Arduino build: the IDE compiles
// only the sketch folder root and src/, never tests/.
// From the PinDex folder, with any C++17 host compiler:
//   g++ -std=c++17 -Wall -Wextra -Itests/mock -I. -DPINDEX_POWERED_PROBING_ENABLED=1
//       tests/test_pindex.cpp probe_engine.cpp dht11.cpp src/drivers/dht11_driver.cpp
//       -o build/test_enabled
//   (repeat with -DPINDEX_POWERED_PROBING_ENABLED=0 -o build/test_disabled)
// Run both; each exits non-zero if any check fails. This file supplies the
// driver registry itself (link seam), so tests can register the real
// Dht11Driver or fake drivers. Shorts, faults and sensors are simulated;
// nothing here touches hardware.

#include <stdio.h>
#include <string.h>

#include <Arduino.h>  // tests/mock/Arduino.h

#include "config.h"
#include "dht11.h"
#include "probe_engine.h"
#include "sensor_driver.h"
#include "src/drivers/dht11_driver.h"
#include "src/drivers/driver_registry.h"

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

static void useDrivers(SensorDriver *first = nullptr, SensorDriver *second = nullptr) {
  testDriverCount = 0;
  if (first != nullptr) testDrivers[testDriverCount++] = first;
  if (second != nullptr) testDrivers[testDriverCount++] = second;
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
  const uint8_t order[PROBE_PERMUTATION_COUNT][3] = {
      {0, 1, 2}, {2, 1, 0}, {1, 0, 2}, {1, 2, 0}, {0, 2, 1}, {2, 0, 1}};
  const char *text[PROBE_PERMUTATION_COUNT] = {
      "A:VCC | B:DAT | C:GND", "A:GND | B:DAT | C:VCC", "A:DAT | B:VCC | C:GND",
      "A:GND | B:VCC | C:DAT", "A:VCC | B:GND | C:DAT", "A:DAT | B:GND | C:VCC"};
  char buf[PROBE_MAPPING_TEXT_LEN];
  for (int i = 0; i < PROBE_PERMUTATION_COUNT; i++) {
    const ProbeMapping &m = PROBE_PERMUTATIONS[i];
    CHECK(m.vcc == order[i][0] && m.data == order[i][1] && m.gnd == order[i][2]);
    formatProbeMapping(m, buf, sizeof buf);
    CHECK(strcmp(buf, text[i]) == 0);
    CHECK(strlen(buf) * 6 <= OLED_WIDTH);  // size-1 font is 6 px per glyph
  }
}

// ---- Simulation helpers ----------------------------------------------------

static const uint8_t kProbeGpio[PROBE_NODE_COUNT] = {PROBE_PIN_A, PROBE_PIN_B, PROBE_PIN_C};
static int lockLeaks = 0;

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

static int permutationOf(uint8_t vcc, uint8_t data, uint8_t gnd) {
  for (int i = 0; i < PROBE_PERMUTATION_COUNT; i++) {
    const ProbeMapping &m = PROBE_PERMUTATIONS[i];
    if (kProbeGpio[m.vcc] == vcc && kProbeGpio[m.data] == data && kProbeGpio[m.gnd] == gnd) {
      return i;
    }
  }
  return -1;
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

// ---- Fake driver: proves the engine is driver-agnostic ---------------------

struct Attempt {
  int driver;
  int perm;
  bool sample;  // false = probe()
  uint64_t us;
};
static Attempt attempts[256];
static int attemptCount = 0;

class FakeDriver final : public SensorDriver {
 public:
  FakeDriver(int id, const char *name) : id_(id), name_(name) {}

  DriverRequirements req = {300, 500, 700, 1000};
  int pendingCalls = 0;  // Pending results before each operation completes
  int acceptPerm = -1;   // probe() succeeds only on this permutation index
  bool samplesSucceed = true;

  int probeStarts = 0;
  int sampleStarts = 0;
  int calls = 0;
  int resets = 0;
  int unsafeResets = 0;     // reset() while a probe pin was not plain INPUT
  int unpoweredStarts = 0;  // operation started without the engine powering VCC/GND

  const char *getName() const override { return name_; }
  DriverOperationStatus getOperationStatus() const override { return status_; }
  const DriverRequirements &getRequirements() const override { return req; }
  const char *getFailureReason() const override {
    return status_ == DriverOperationStatus::Failed ? "fake-failed" : "";
  }

  bool probe(uint8_t pinVcc, uint8_t pinData, uint8_t pinGnd) override {
    calls++;
    if (status_ != DriverOperationStatus::Pending) {
      probeStarts++;
      perm_ = permutationOf(pinVcc, pinData, pinGnd);
      bound_ = false;
      start(false, pinVcc, pinGnd);
    }
    if (remaining_ > 0) {
      remaining_--;
      return false;
    }
    bound_ = perm_ == acceptPerm;
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
      start(true, vccPin_, gndPin_);
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
  void start(bool sample, uint8_t vcc, uint8_t gnd) {
    vccPin_ = vcc;
    gndPin_ = gnd;
    status_ = DriverOperationStatus::Pending;
    remaining_ = pendingCalls;
    if (!(mock::drivesHigh(vcc) && mock::drivesLow(gnd))) unpoweredStarts++;
    if (attemptCount < 256) attempts[attemptCount++] = Attempt{id_, perm_, sample, mock::nowUs};
  }

  int id_;
  const char *name_;
  DriverOperationStatus status_ = DriverOperationStatus::Idle;
  bool bound_ = false;
  int remaining_ = 0;
  int perm_ = -1;
  uint8_t vccPin_ = 0;
  uint8_t gndPin_ = 0;
};

#if PINDEX_POWERED_PROBING_ENABLED

// ---- Simulated DHT11 and socket faults -------------------------------------

struct SimSensor {
  bool attached = false;
  ProbeMapping wiring{0, 1, 2};
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

static Dht11Driver dht;  // the production driver, registered per test
static SimSensor sensor;
static bool shortAB = false;  // A and B behave as one node
static bool flakyA = false;   // A alternates HIGH/LOW while pulled up
static int flakyReads = 0;
static int lockedPinModes = 0;

static uint8_t gpioOf(uint8_t node) { return kProbeGpio[node]; }

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

// Called after every pinMode()/digitalWrite(): tracks power and start pulses.
static void onPinChange(uint8_t, bool isPinMode) {
  if (isPinMode && mock::criticalDepth != 0) lockedPinModes++;
  if (!sensor.attached) return;

  const bool poweredNow = mock::drivesHigh(gpioOf(sensor.wiring.vcc)) &&
                          mock::drivesLow(gpioOf(sensor.wiring.gnd));
  if (poweredNow && !sensor.powered) sensor.poweredSinceUs = mock::nowUs;
  if (!poweredNow) sensor.responding = false;
  sensor.powered = poweredNow;

  const bool lowNow = mock::drivesLow(gpioOf(sensor.wiring.data));
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
  const uint8_t a = gpioOf(0);
  const uint8_t b = gpioOf(1);
  if (shortAB && (pin == a || pin == b)) {
    return ((mock::pins[a].mode | mock::pins[b].mode) & PULLUP) ? HIGH : LOW;
  }
  if (flakyA && pin == a && mock::pins[a].mode == INPUT_PULLUP) {
    return (flakyReads++ % 2 == 0) ? HIGH : LOW;
  }
  if (sensor.attached && pin == gpioOf(sensor.wiring.data)) {
    if (sensor.stuckLow) return LOW;
    if (sensor.responding) return responseLevel(mock::nowUs - sensor.responseStartUs);
  }
  return -1;
}

// Call useDrivers() first: init resets every registered driver.
static void resetWorld(uint64_t startUs = 1000000ULL) {
  mock::reset(startUs);
  sensor = SimSensor();
  shortAB = false;
  flakyA = false;
  flakyReads = 0;
  attemptCount = 0;
  mock::readHook = readHook;
  mock::changeHook = onPinChange;
  initProbeEngine();
}

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

// ---- Event-log replay: one Session per powered interval --------------------

struct Session {
  uint64_t startUs = 0;
  uint64_t endUs = UINT64_MAX;  // UINT64_MAX = still powered
  int vcc = -1;
  int gnd = -1;
  int data = -1;  // known only for drivers that pull DATA low (DHT)
  int starts = 0;
  uint64_t firstStartUs = 0;
};

struct SessionLog {
  Session s[64];
  int count = 0;
  int violations = 0;
  uint64_t startTimes[256] = {};
  int startCount = 0;
};

// Replays every probe pin change and flags sequencing violations: anything
// driven while precheck pulls remain, two supplies, VCC without GND driven
// low (covers GND-first power-up and GND-last teardown), DATA not released
// before VCC, or DATA driven high push-pull (shows up as two supplies).
static SessionLog collectSessions() {
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
      for (int k = 0; k < PROBE_NODE_COUNT; k++) {
        if (k != n && mode[k] != INPUT) log.violations++;
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
      if (odNode == n && !odBefore) {  // a new DHT start request
        if (cur.starts++ == 0) cur.firstStartUs = e.us;
        if (log.startCount < 256) log.startTimes[log.startCount++] = e.us;
      }
    }
    if (!driven && active) {
      active = false;
      cur.endUs = e.us;
      if (log.count < 64) log.s[log.count++] = cur;
    }
  }
  if (active && log.count < 64) log.s[log.count++] = cur;
  return log;
}

static bool sessionUses(const Session &s, const ProbeMapping &m) {
  return s.vcc == m.vcc && s.gnd == m.gnd && s.data == m.data;
}

static bool sessionPowers(const Session &s, const ProbeMapping &m) {
  return s.vcc == m.vcc && s.gnd == m.gnd;
}

// ---- DHT11 through the engine (Phase 2 behaviour, preserved) ---------------

static void testEmptySocketScan() {
  resetWithDht();
  const uint64_t t0 = mock::nowUs;
  CHECK(runUntil([] { return phase() == ProbePhase::NoComponent; }, 20000));
  CHECK(same(probeStatus().lastError, dhtErrorName(DhtError::NoResponse)));
  CHECK(probeStatus().precheck == PrecheckResult::NoObviousCoupling);
  CHECK(!probeStatus().hasMapping && !probeStatus().readingValid);
  CHECK(probeStatus().componentName == nullptr);
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

  // The retry powers only the saved mapping, after cooldown, precheck and a
  // fresh stabilization, and acquisition starts keep their minimum spacing.
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

  // Discovery resumes only after the rescan interval.
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

static void testPrecheckRejects() {
  resetWithDht();
  shortAB = true;  // simulated A-B short
  runMs(1000);
  CHECK(phase() == ProbePhase::Fault);
  CHECK(probeStatus().precheck == PrecheckResult::SuspiciousCoupling);
  CHECK(allProbesPlainInput());
  CHECK(collectSessions().count == 0);  // nothing was ever powered
  runMs(20000);                         // latched: no automatic retry
  CHECK(phase() == ProbePhase::Fault && collectSessions().count == 0);

  CHECK(probeRequestRetry());  // fault still present: rejected again
  runMs(1000);
  CHECK(phase() == ProbePhase::Fault && collectSessions().count == 0);

  shortAB = false;  // fixed: retry reruns the whole precheck, then scans
  CHECK(probeRequestRetry());
  CHECK(!probeRequestRetry());  // only valid from Fault
  runMs(2000);
  CHECK(phase() == ProbePhase::Scanning);
  CHECK(probeStatus().precheck == PrecheckResult::NoObviousCoupling);
  CHECK(collectSessions().count >= 1 && collectSessions().violations == 0);

  resetWithDht();
  flakyA = true;  // inconsistent readings also veto
  runMs(1000);
  CHECK(phase() == ProbePhase::Fault);
  CHECK(probeStatus().precheck == PrecheckResult::InconsistentReadings);
  CHECK(collectSessions().count == 0);

  // A precheck fault during a retry latches at once (no three-failure wait).
  CHECK(detectWith(3));
  sensor.respond = false;
  CHECK(runUntil([] { return probeStatus().consecutiveFailures == 1; }, 3000));
  shortAB = true;
  CHECK(runUntil([] { return phase() == ProbePhase::Fault; }, 3000));
  CHECK(!probeStatus().hasMapping && !probeStatus().detected);
  CHECK(probeStatus().componentName == nullptr);
  CHECK(allProbesPlainInput());

  // A rejected socket is never handed to any driver.
  FakeDriver fake(1, "Fake A");
  fake.acceptPerm = 0;
  useDrivers(&fake);
  resetWorld();
  shortAB = true;
  runMs(2000);
  CHECK(phase() == ProbePhase::Fault && fake.calls == 0);
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

// ---- Dht11Driver contract, driven directly ---------------------------------

static void powerByHand(const ProbeMapping &m) {  // the engine's job, done manually here
  resetTestPins();
  digitalWrite(gpioOf(m.gnd), LOW);
  pinMode(gpioOf(m.gnd), OUTPUT);
  digitalWrite(gpioOf(m.vcc), HIGH);
  pinMode(gpioOf(m.vcc), OUTPUT);
  pinMode(gpioOf(m.data), INPUT_PULLUP);
  mock::nowUs += DHT_POWER_STABILIZE_MS * 1000ULL;
}

static void testDht11DriverContract() {
  useDrivers();  // the engine stays idle; this test drives the driver itself
  resetWorld();
  Dht11Driver d;
  char l1[PROBE_TELEMETRY_TEXT_LEN];
  char l2[PROBE_TELEMETRY_TEXT_LEN];
  const uint8_t vcc = gpioOf(0), data = gpioOf(1), gnd = gpioOf(2);
  CHECK(same(d.getName(), "DHT11 Temp & Hum"));
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

  // Probe: Pending for the 20 ms start pulse, then one genuine frame. The
  // driver touches only DATA.
  sensor.attached = true;
  sensor.wiring = PROBE_PERMUTATIONS[0];
  powerByHand(PROBE_PERMUTATIONS[0]);
  d.reset();
  const size_t eventsBefore = mock::eventCount;
  int pending = 0;
  while (!d.probe(vcc, data, gnd) && pending < 100) {
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
  CHECK(!d.probe(vcc, data, gnd));
  CHECK(d.getOperationStatus() == DriverOperationStatus::Failed);
  CHECK(same(d.getFailureReason(), "contract-violation"));
  CHECK(mock::pins[data].mode == INPUT);

  // A failed sample clears the outputs.
  powerByHand(PROBE_PERMUTATIONS[0]);
  d.reset();
  while (!d.probe(vcc, data, gnd) && d.getOperationStatus() == DriverOperationStatus::Pending) {
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
  CHECK(!d.probe(vcc, data, gnd));
  CHECK(!d.probe(gnd, data, vcc));
  CHECK(d.getOperationStatus() == DriverOperationStatus::Failed);

  // reset() during a start pulse only releases DATA to plain INPUT; after the
  // engine has made the pins safe, reset() (twice) touches nothing.
  powerByHand(PROBE_PERMUTATIONS[0]);
  d.reset();
  CHECK(!d.probe(vcc, data, gnd));
  CHECK(mock::drivesLow(data));
  d.reset();
  CHECK(mock::pins[data].mode == INPUT);
  CHECK(d.getOperationStatus() == DriverOperationStatus::Idle);
  resetTestPins();
  const size_t quiet = mock::eventCount;
  d.reset();
  d.reset();
  CHECK(mock::eventCount == quiet);
  CHECK(mock::criticalDepth == 0);
}

// ---- Engine architecture with fake drivers ---------------------------------

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
    CHECK(attempts[i].perm == i / 2 && attempts[i].driver == 1 + i % 2 && !attempts[i].sample);
  }
  CHECK(a.sampleStarts == 0 && b.sampleStarts == 0);
  CHECK(a.unpoweredStarts == 0 && b.unpoweredStarts == 0);  // the engine powered each attempt
  CHECK(a.resets >= 1 + PROBE_PERMUTATION_COUNT && b.resets >= 1 + PROBE_PERMUTATION_COUNT);
  CHECK(a.unsafeResets == 0 && b.unsafeResets == 0);  // pins were safe before every reset

  // Every attempt, even on the same permutation, gets its own power cycle,
  // precheck and power-off gap, and waits for the driver's stabilization.
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
  b.acceptPerm = 0;
  useDrivers(&a, &b);
  resetWorld();
  CHECK(runUntil([] { return phase() == ProbePhase::Confirming; }, 10000));
  CHECK(same(probeStatus().componentName, "Fake B"));
  CHECK(a.probeStarts == 1 && b.probeStarts == 1);
  CHECK(probeStatus().mapping.vcc == PROBE_PERMUTATIONS[0].vcc &&
        probeStatus().mapping.gnd == PROBE_PERMUTATIONS[0].gnd);
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
  a.acceptPerm = 0;
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
  stuck.acceptPerm = 0;
  stuck.pendingCalls = 1000000;
  next.acceptPerm = 0;
  useDrivers(&stuck, &next);
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
  a.acceptPerm = 0;
  a.req = {250, 600, 200, 1000};  // sample interval below the minimum: the minimum wins
  useDrivers(&a);
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
  b.acceptPerm = 2;
  useDrivers(&a, &b);
  resetWorld();
  CHECK(runUntil([] { return phase() == ProbePhase::Detected; }, 20000));
  const int detectedAt = attemptCount;
  char shown[PROBE_TELEMETRY_TEXT_LEN];
  snprintf(shown, sizeof shown, "%s", probeStatus().telemetry1);

  b.acceptPerm = -1;  // component pulled out
  b.samplesSucceed = false;
  CHECK(runUntil([] { return probeStatus().consecutiveFailures == 1; }, 2000));
  CHECK(phase() == ProbePhase::Retrying && same(probeStatus().telemetry1, shown));
  CHECK(runUntil([] { return phase() == ProbePhase::Idle; }, 10000));
  CHECK(probeStatus().componentName == nullptr && !probeStatus().hasMapping);
  CHECK(!probeStatus().detected && !probeStatus().readingValid);
  CHECK(probeStatus().telemetry1[0] == '\0' && probeStatus().telemetry2[0] == '\0');
  CHECK(allProbesPlainInput() && a.unsafeResets == 0 && b.unsafeResets == 0);
  for (int i = detectedAt; i < attemptCount; i++) {  // recovery used only the saved driver
    CHECK(attempts[i].driver == 2 && attempts[i].perm == 2);
  }

  // After the rescan interval, discovery restarts from the first driver.
  const int before = attemptCount;
  runMs(PROBE_RESCAN_INTERVAL_MS + 1000);
  CHECK(attemptCount > before && attempts[before].driver == 1 && attempts[before].perm == 0);
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

#else  // !PINDEX_POWERED_PROBING_ENABLED

static void testDisabledNeverDrives() {
  FakeDriver a(1, "Fake A");
  FakeDriver b(2, "Fake B");
  a.acceptPerm = 0;
  b.acceptPerm = 0;
  useDrivers(&a, &b);
  mock::reset(1000000ULL);
  attemptCount = 0;
  initProbeEngine();
  const uint32_t rev = probeStatus().revision;
  runMs(30000);
  CHECK(phase() == ProbePhase::Disabled);
  CHECK(probeStatus().revision == rev);
  CHECK(!probeRequestRetry());  // a retry never bypasses the setting
  runMs(1000);
  CHECK(phase() == ProbePhase::Disabled);
  CHECK(a.calls == 0 && b.calls == 0 && attemptCount == 0);  // no driver ever runs

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
  testFrameDecoding();
  testPulseDecoding();
  testMappingText();
#if PINDEX_POWERED_PROBING_ENABLED
  testEmptySocketScan();
  testDetectionNeedsTwoFrames();
  testStaleThenRecovery();
  testRemovalAfterThreeFailures();
  testCaptureErrors();
  testPrecheckRejects();
  testRollover();
  testDht11DriverContract();
  testPermutationMajorDriverOrder();
  testLaterDriverAndConfirmation();
  testPendingIsNotFailure();
  testRequirementsDriveScheduling();
  testRemovalClearsActiveDriver();
  testEmptyRegistry();
  CHECK(lockedPinModes == 0);  // pinMode() never ran inside the capture lock
#else
  testDisabledNeverDrives();
#endif
  CHECK(lockLeaks == 0);  // the capture lock was always released
  printf("%s build: %d checks, %d failed\n",
         PINDEX_POWERED_PROBING_ENABLED ? "powered-probing" : "probing-disabled", checks, failures);
  return failures == 0 ? 0 : 1;
}
