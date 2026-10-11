// Host tests for PinDex Phase 2 logic. Not part of the Arduino build: the
// IDE compiles only the sketch folder root (and src/), never tests/.
// From the PinDex folder, with any C++17 host compiler:
//   g++ -std=c++17 -Wall -Wextra -Itests/mock -I. -DPINDEX_POWERED_PROBING_ENABLED=1
//       tests/test_pindex.cpp probe_engine.cpp dht11.cpp -o build/test_enabled
//   g++ -std=c++17 -Wall -Wextra -Itests/mock -I. -DPINDEX_POWERED_PROBING_ENABLED=0
//       tests/test_pindex.cpp probe_engine.cpp dht11.cpp -o build/test_disabled
// Run both; each exits non-zero if any check fails. Shorts, faults and the
// sensor are simulated; nothing here touches hardware.

#include <stdio.h>
#include <string.h>

#include <Arduino.h>  // tests/mock/Arduino.h

#include "config.h"
#include "dht11.h"
#include "probe_engine.h"

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

static void resetWorld(uint64_t startUs = 1000000ULL) {
  mock::reset(startUs);
  sensor = SimSensor();
  shortAB = false;
  flakyA = false;
  flakyReads = 0;
  mock::readHook = readHook;
  mock::changeHook = onPinChange;
  initProbeEngine();
}

static bool detectWith(int wiring) {
  resetWorld();
  sensor.attached = true;
  sensor.wiring = PROBE_PERMUTATIONS[wiring];
  return runUntil([] { return probeStatus().phase == ProbePhase::Detected; }, 20000);
}

// ---- Event-log replay: one Session per powered interval --------------------

struct Session {
  uint64_t startUs = 0;
  uint64_t endUs = UINT64_MAX;  // UINT64_MAX = still powered
  int vcc = -1;
  int gnd = -1;
  int data = -1;
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
      if (odNode == n && !odBefore) {  // a new start request
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

// ---- Powered-path tests ----------------------------------------------------

static void testEmptySocketScan() {
  resetWorld();
  const uint64_t t0 = mock::nowUs;
  CHECK(runUntil([] { return probeStatus().phase == ProbePhase::NoComponent; }, 20000));
  CHECK(probeStatus().lastError == DhtError::NoResponse);
  CHECK(probeStatus().precheck == PrecheckResult::NoObviousCoupling);
  CHECK(!probeStatus().hasMapping && !probeStatus().readingValid);
  CHECK(allProbesPlainInput());

  // Latency is dominated by per-permutation stabilization (~1.46 s each).
  const uint64_t scanMs = (mock::nowUs - t0) / 1000;
  CHECK(scanMs >= 6 * PROBE_POWER_STABILIZE_MS && scanMs <= 6 * 1600);

  const SessionLog log = collectSessions();
  CHECK(log.violations == 0);
  CHECK(log.count == PROBE_PERMUTATION_COUNT);
  for (int i = 0; i < log.count && i < PROBE_PERMUTATION_COUNT; i++) {
    const Session &x = log.s[i];
    CHECK(sessionUses(x, PROBE_PERMUTATIONS[i]));  // exact table order
    CHECK(x.endUs != UINT64_MAX && x.starts == 1);
    CHECK(x.firstStartUs - x.startUs >= (PROBE_POWER_STABILIZE_MS - 1) * 1000);
    if (i > 0) {  // torn down, then the power-off interval, before the next one
      CHECK(x.startUs - log.s[i - 1].endUs >= (PROBE_POWER_OFF_MS - 1) * 1000);
    }
  }

  // Probes stay INPUT for the rescan interval, then discovery restarts.
  runMs(PROBE_RESCAN_INTERVAL_MS - 50);
  CHECK(probeStatus().phase == ProbePhase::NoComponent);
  CHECK(collectSessions().count == PROBE_PERMUTATION_COUNT);
  runMs(100);
  CHECK(probeStatus().phase == ProbePhase::Scanning && probeStatus().scanIndex == 1);
}

static void testDetectionNeedsTwoFrames() {
  resetWorld();
  sensor.attached = true;
  sensor.wiring = PROBE_PERMUTATIONS[3];  // B = VCC, C = DATA, A = GND
  CHECK(runUntil([] { return probeStatus().phase == ProbePhase::Confirming; }, 20000));
  CHECK(sensor.responses == 1);
  CHECK(probeStatus().hasMapping && !probeStatus().detected);

  CHECK(runUntil([] { return probeStatus().phase == ProbePhase::Detected; }, 3000));
  CHECK(sensor.responses == 2);
  CHECK(probeStatus().temperatureDeciC == 250 && probeStatus().humidityPct == 60);
  CHECK(!probeStatus().readingStale && probeStatus().consecutiveFailures == 0);
  CHECK(probeStatus().lastError == DhtError::None);
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
  CHECK(probeStatus().humidityPct == 61 && probeStatus().temperatureDeciC == 260);
  CHECK(probeStatus().lastReadingMs - lastMs >= DHT_READ_INTERVAL_MS - 50);
  CHECK(probeStatus().revision != rev);
  log = collectSessions();
  CHECK(log.count == 4 && log.violations == 0);

  // Identical readings are not a display change.
  const uint32_t quietRev = probeStatus().revision;
  runMs(2 * DHT_READ_INTERVAL_MS);
  CHECK(probeStatus().revision == quietRev);
  CHECK(probeStatus().phase == ProbePhase::Detected);
}

static void testStaleThenRecovery() {
  CHECK(detectWith(3));
  sensor.respond = false;  // e.g. a bad contact for one read
  CHECK(runUntil([] { return probeStatus().consecutiveFailures == 1; }, 3000));
  CHECK(probeStatus().phase == ProbePhase::Retrying);
  CHECK(probeStatus().readingStale && probeStatus().readingValid && probeStatus().detected);
  CHECK(probeStatus().temperatureDeciC == 250);  // old value kept, marked stale
  CHECK(probeStatus().lastError == DhtError::NoResponse);
  CHECK(allProbesPlainInput());  // torn down at once

  sensor.respond = true;
  CHECK(runUntil([] { return probeStatus().phase == ProbePhase::Detected; }, 5000));
  CHECK(!probeStatus().readingStale && probeStatus().consecutiveFailures == 0);

  // The retry powers only the saved mapping, after cooldown, precheck and a
  // fresh stabilization, and start requests keep their minimum spacing.
  const SessionLog log = collectSessions();
  CHECK(log.violations == 0);
  CHECK(log.count == 5);
  CHECK(sessionUses(log.s[4], PROBE_PERMUTATIONS[3]));
  CHECK(log.s[4].startUs - log.s[3].endUs >= (PROBE_RETRY_COOLDOWN_MS - 1) * 1000);
  CHECK(log.s[4].firstStartUs - log.s[4].startUs >= (PROBE_POWER_STABILIZE_MS - 1) * 1000);
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
  CHECK(probeStatus().phase == ProbePhase::Retrying && probeStatus().readingStale);
  CHECK(runUntil([] { return probeStatus().consecutiveFailures == 2; }, 4000));
  CHECK(probeStatus().hasMapping);
  CHECK(runUntil([] { return probeStatus().phase == ProbePhase::Idle; }, 4000));
  CHECK(!probeStatus().hasMapping && !probeStatus().detected);
  CHECK(!probeStatus().readingValid && !probeStatus().readingStale);
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
  CHECK(probeStatus().phase == ProbePhase::Idle);
  CHECK(collectSessions().count == log.count);
  runMs(100);
  CHECK(probeStatus().phase == ProbePhase::Scanning && probeStatus().scanIndex == 1);
}

// Runs permutation 1 against a misbehaving sensor wired as permutation 1 and
// returns the error that rejected it.
static DhtError firstAttemptError(void (*configure)()) {
  resetWorld();
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
  CHECK(firstAttemptError([] { sensor.stuckLow = true; }) == DhtError::StuckLow);
  CHECK(firstAttemptError([] { sensor.respond = false; }) == DhtError::NoResponse);  // stuck high
  CHECK(firstAttemptError([] { sensor.ackLowUs = 20; }) == DhtError::BadAckTiming);
  CHECK(firstAttemptError([] { sensor.ackLowUs = 200; }) == DhtError::AckTimeout);
  CHECK(firstAttemptError([] { sensor.bitsSent = 20; }) == DhtError::BitTimeout);  // truncated
  CHECK(firstAttemptError([] { sensor.oneHighUs = 110; }) == DhtError::BadBitTiming);
  CHECK(firstAttemptError([] { sensor.frame[4] ^= 1; }) == DhtError::Checksum);
  CHECK(firstAttemptError([] {
          const uint8_t f[5] = {60, 5, 25, 0, 90};
          memcpy(sensor.frame, f, 5);
        }) == DhtError::UnsupportedEncoding);

  // Every phase in range but the frame too slow overall: the whole-frame
  // deadline still ends the capture, bounding the time interrupts are masked.
  CHECK(firstAttemptError([] {
          sensor.bitLowUs = 80;
          sensor.oneHighUs = 100;
          memset(sensor.frame, 0xFF, 5);
        }) == DhtError::BitTimeout);
  CHECK(mock::criticalMaxUs > 0 && mock::criticalMaxUs <= DHT_FRAME_DEADLINE_US + 10);
}

static void testPrecheckRejects() {
  resetWorld();
  shortAB = true;  // simulated A-B short
  runMs(1000);
  CHECK(probeStatus().phase == ProbePhase::Fault);
  CHECK(probeStatus().precheck == PrecheckResult::SuspiciousCoupling);
  CHECK(allProbesPlainInput());
  CHECK(collectSessions().count == 0);  // nothing was ever powered
  runMs(20000);                         // latched: no automatic retry
  CHECK(probeStatus().phase == ProbePhase::Fault && collectSessions().count == 0);

  CHECK(probeRequestRetry());  // fault still present: rejected again
  runMs(1000);
  CHECK(probeStatus().phase == ProbePhase::Fault && collectSessions().count == 0);

  shortAB = false;  // fixed: retry reruns the whole precheck, then scans
  CHECK(probeRequestRetry());
  CHECK(!probeRequestRetry());  // only valid from Fault
  runMs(2000);
  CHECK(probeStatus().phase == ProbePhase::Scanning);
  CHECK(probeStatus().precheck == PrecheckResult::NoObviousCoupling);
  CHECK(collectSessions().count >= 1 && collectSessions().violations == 0);

  resetWorld();
  flakyA = true;  // inconsistent readings also veto
  runMs(1000);
  CHECK(probeStatus().phase == ProbePhase::Fault);
  CHECK(probeStatus().precheck == PrecheckResult::InconsistentReadings);
  CHECK(collectSessions().count == 0);

  // A precheck fault during a retry latches at once (no three-failure wait).
  CHECK(detectWith(3));
  sensor.respond = false;
  CHECK(runUntil([] { return probeStatus().consecutiveFailures == 1; }, 3000));
  shortAB = true;
  CHECK(runUntil([] { return probeStatus().phase == ProbePhase::Fault; }, 3000));
  CHECK(!probeStatus().hasMapping && !probeStatus().detected);
  CHECK(allProbesPlainInput());
}

static void testRollover() {
  // millis() wraps ~600 ms in, during the first power-up stabilization.
  resetWorld((static_cast<uint64_t>(0xFFFFFFFFu) - 600) * 1000);
  sensor.attached = true;
  sensor.wiring = PROBE_PERMUTATIONS[0];
  CHECK(runUntil([] { return probeStatus().phase == ProbePhase::Detected; }, 5000));
  const SessionLog log = collectSessions();
  CHECK(log.count == 1 && log.violations == 0);
  CHECK(log.s[0].firstStartUs - log.s[0].startUs >= (PROBE_POWER_STABILIZE_MS - 1) * 1000);
  CHECK(log.s[0].firstStartUs - log.s[0].startUs <= (PROBE_POWER_STABILIZE_MS + 2) * 1000);
  CHECK(millis() < 10000);  // the counter really wrapped

  // micros() wraps inside every capture.
  resetWorld();
  sensor.attached = true;
  sensor.wiring = PROBE_PERMUTATIONS[0];
  sensor.wrapMicrosAtRelease = true;
  CHECK(runUntil([] { return probeStatus().phase == ProbePhase::Detected; }, 5000));
  CHECK(probeStatus().temperatureDeciC == 250 && probeStatus().humidityPct == 60);
}

#else  // !PINDEX_POWERED_PROBING_ENABLED

static void testDisabledNeverDrives() {
  mock::reset(1000000ULL);
  initProbeEngine();
  const uint32_t rev = probeStatus().revision;
  runMs(30000);
  CHECK(probeStatus().phase == ProbePhase::Disabled);
  CHECK(probeStatus().revision == rev);
  CHECK(!probeRequestRetry());  // a retry never bypasses the setting
  runMs(1000);
  CHECK(probeStatus().phase == ProbePhase::Disabled);

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
  CHECK(lockedPinModes == 0);  // pinMode() never ran inside the capture lock
#else
  testDisabledNeverDrives();
#endif
  CHECK(lockLeaks == 0);  // the capture lock was always released
  printf("%s build: %d checks, %d failed\n",
         PINDEX_POWERED_PROBING_ENABLED ? "powered-probing" : "probing-disabled", checks, failures);
  return failures == 0 ? 0 : 1;
}
