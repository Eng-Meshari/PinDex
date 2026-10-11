#include "probe_engine.h"

#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include "config.h"

// Safety scope: plain INPUT (high impedance) only means the firmware is not
// driving the probes. It is NOT protection against shorts, overvoltage, ESD
// or charged capacitors. Powered probing drives probe GPIOs as a nominal
// 3.3V logic-level supply, not a regulated rail, and must only be enabled
// behind a reviewed protected interface (see README).

static_assert(DHT_READ_INTERVAL_MS >= DHT_MIN_START_INTERVAL_MS,
              "scheduled reads would violate the start-request spacing");
static_assert(PROBE_RETRY_COOLDOWN_MS >= PROBE_POWER_OFF_MS,
              "a retry must include the power-off interval");
static_assert(PROBE_RESCAN_INTERVAL_MS >= PROBE_POWER_OFF_MS,
              "a rescan must include the power-off interval");

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
  Stabilizing,   // PROBE_POWER_STABILIZE_MS before the first start request
  StartLow,      // DATA held low for DHT_START_LOW_MS
  Capture,       // release DATA and time the response (bounded busy poll)
  Validate,      // decode the captured pulses
  Confirming,    // candidate powered, waiting to request the next frame
  Monitoring,    // detected, powered, reading every DHT_READ_INTERVAL_MS
  PowerOffWait,  // discovery: torn down, waiting before the next permutation
  RetryWait,     // saved mapping failed: torn down, cooling down
  Fault,         // precheck rejected the socket; latched, probes INPUT
};

State state = State::Disabled;
uint32_t stateSinceMs = 0;  // entry time of the state (or precheck step)
uint32_t idleDelayMs = 0;
uint32_t lastStartMs = 0;   // last start request (DATA pulled low)
uint8_t permIndex = 0;      // discovery permutation, 0-based
uint8_t confirmations = 0;  // consecutive valid frames on the saved mapping
bool noComponent = false;   // last discovery found nothing
uint8_t precheckSource = 0;
uint8_t precheckSample = 0;
uint8_t precheckFirst[PROBE_NODE_COUNT];
DhtPulses pulses;
DhtError captureError = DhtError::None;
ProbeStatus status;
portMUX_TYPE captureMux = portMUX_INITIALIZER_UNLOCKED;

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
  status.hasMapping = false;
  status.mapping = ProbeMapping{0, 0, 0};
  status.detected = false;
  status.readingValid = false;
  status.readingStale = false;
  status.temperatureDeciC = 0;
  status.humidityPct = 0;
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

void beginStartRequest(uint32_t now) {
  const uint8_t pin = gpio(activeMapping().data);
  // Open-drain with pull-up: LOW sinks the line, HIGH only releases it, so
  // DATA is never driven high. Latch LOW before enabling (see driveNode).
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT_OPEN_DRAIN | PULLUP);
  lastStartMs = now;
  enter(State::StartLow, now);
}

// ---- Bounded DHT response capture ----------------------------------------

constexpr uint32_t kTimedOut = 0xFFFFFFFFUL;

// Masks interrupts on this core for the capture only; the destructor
// restores the previous state on every return path. Never held across the
// start pulse, stabilization, retries, Serial or display work.
struct CaptureCriticalSection {
  CaptureCriticalSection() { portENTER_CRITICAL(&captureMux); }
  ~CaptureCriticalSection() { portEXIT_CRITICAL(&captureMux); }
};

// How long the line stays at `level`, in us. kTimedOut if the phase exceeds
// DHT_PHASE_TIMEOUT_US or the whole response exceeds DHT_FRAME_DEADLINE_US.
uint32_t levelWidthUs(uint8_t pin, int level, uint32_t frameStartUs) {
  const uint32_t startUs = micros();
  for (;;) {
    const uint32_t nowUs = micros();
    if (digitalRead(pin) != level) return nowUs - startUs;
    if (nowUs - startUs > DHT_PHASE_TIMEOUT_US || nowUs - frameStartUs > DHT_FRAME_DEADLINE_US) {
      return kTimedOut;
    }
  }
}

// Releases DATA and records one response. Busy-polls for at most
// DHT_FRAME_DEADLINE_US plus one poll (~4-5 ms for a normal frame) with
// interrupts masked on this core. No Serial, display or allocation here.
DhtError captureFrame(uint8_t pin, DhtPulses &p) {
  memset(&p, 0, sizeof p);
  CaptureCriticalSection lock;

  digitalWrite(pin, HIGH);  // open-drain release; the pull-up raises DATA
  const uint32_t frameStartUs = micros();

  if (levelWidthUs(pin, LOW, frameStartUs) == kTimedOut) return DhtError::StuckLow;
  if (levelWidthUs(pin, HIGH, frameStartUs) == kTimedOut) return DhtError::NoResponse;

  uint32_t w = levelWidthUs(pin, LOW, frameStartUs);
  if (w == kTimedOut) return DhtError::AckTimeout;
  p.ackLowUs = static_cast<uint16_t>(w);
  w = levelWidthUs(pin, HIGH, frameStartUs);
  if (w == kTimedOut) return DhtError::AckTimeout;
  p.ackHighUs = static_cast<uint16_t>(w);

  // Each bit is a LOW phase then a HIGH phase; the last HIGH ends when the
  // sensor pulls low for end-of-frame, so it is also bounded.
  for (uint8_t i = 0; i < DHT_FRAME_BITS; i++) {
    w = levelWidthUs(pin, LOW, frameStartUs);
    if (w == kTimedOut) return DhtError::BitTimeout;
    p.bitLowUs[i] = static_cast<uint16_t>(w);
    w = levelWidthUs(pin, HIGH, frameStartUs);
    if (w == kTimedOut) return DhtError::BitTimeout;
    p.bitHighUs[i] = static_cast<uint16_t>(w);
  }
  return DhtError::None;
}

// ---- Results ---------------------------------------------------------------

void onValidFrame(const DhtReading &reading, uint32_t now) {
  status.temperatureDeciC = reading.temperatureDeciC;
  status.humidityPct = reading.humidityPct;
  status.readingValid = true;
  status.readingStale = false;
  status.lastReadingMs = now;
  status.consecutiveFailures = 0;

  if (!status.hasMapping) {  // first valid frame: save candidate, stop scanning
    status.hasMapping = true;
    status.mapping = PROBE_PERMUTATIONS[permIndex];
    confirmations = 0;
  }
  if (!status.detected && ++confirmations >= PROBE_CONFIRM_FRAMES) {
    status.detected = true;
  }
  // The mapping stays powered for the next scheduled read.
  enter(status.detected ? State::Monitoring : State::Confirming, now);
}

void onReadFailure(uint32_t now) {
  powerDown(activeMapping());  // immediately, before any decision

  if (!status.hasMapping) {  // discovery: this permutation failed
    if (++permIndex < PROBE_PERMUTATION_COUNT) {
      enter(State::PowerOffWait, now);
    } else {
      noComponent = true;  // "No Component / Unknown"
      goIdle(now, PROBE_RESCAN_INTERVAL_MS);
    }
    return;
  }

  status.readingStale = true;
  confirmations = 0;  // confirmation needs consecutive frames
  if (++status.consecutiveFailures < PROBE_MAX_CONSECUTIVE_FAILURES) {
    enter(State::RetryWait, now);  // retry only the saved mapping
    return;
  }
  // No removal sensor exists: removal is inferred from failed reads.
  clearResult();
  goIdle(now, PROBE_RESCAN_INTERVAL_MS);
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

    case State::Stabilizing:
      // Retries of the saved mapping also respect the start-request spacing.
      if (elapsed(now, stateSinceMs, PROBE_POWER_STABILIZE_MS) &&
          (!status.hasMapping || elapsed(now, lastStartMs, DHT_MIN_START_INTERVAL_MS))) {
        beginStartRequest(now);
      }
      break;

    case State::StartLow:
      if (!elapsed(now, stateSinceMs, DHT_START_LOW_MS)) break;
      enter(State::Capture, now);
      [[fallthrough]];  // release and capture in this tick so the reply isn't missed

    case State::Capture: {
      const uint8_t pin = gpio(activeMapping().data);
      captureError = captureFrame(pin, pulses);
      pinMode(pin, INPUT_PULLUP);  // plain pulled-up input again, outside the lock
      enter(State::Validate, now);
      break;
    }

    case State::Validate: {
      DhtError error = captureError;
      uint8_t bytes[DHT_FRAME_BYTES] = {};
      DhtReading reading{};
      if (error == DhtError::None) error = dhtPulsesToBytes(pulses, bytes);
      if (error == DhtError::None) error = dhtDecodeFrame(bytes, reading);
      status.lastError = error;
      if (error == DhtError::None) {
        onValidFrame(reading, now);
      } else {
        onReadFailure(now);
      }
      break;
    }

    case State::Confirming:
    case State::Monitoring:
      if (elapsed(now, lastStartMs, DHT_READ_INTERVAL_MS)) beginStartRequest(now);
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

bool sameForDisplay(const ProbeStatus &a, const ProbeStatus &b) {
  return a.phase == b.phase && a.scanIndex == b.scanIndex && a.hasMapping == b.hasMapping &&
         a.mapping.vcc == b.mapping.vcc && a.mapping.data == b.mapping.data &&
         a.mapping.gnd == b.mapping.gnd && a.detected == b.detected &&
         a.readingValid == b.readingValid && a.readingStale == b.readingStale &&
         a.temperatureDeciC == b.temperatureDeciC && a.humidityPct == b.humidityPct &&
         a.consecutiveFailures == b.consecutiveFailures && a.lastError == b.lastError &&
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
  const ProbeStatus before = status;
  const uint32_t now = millis();

  status = ProbeStatus{};
  status.revision = before.revision + 1;  // force a first redraw
  noComponent = false;
  permIndex = 0;
  captureError = DhtError::None;

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
  startDiscovery(millis());  // full precheck, then permutation 1
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
