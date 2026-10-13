#include "binary_driver.h"

#include <Arduino.h>
#include <stdio.h>

// probe(), after the engine has powered the mapping (GND LOW, VCC HIGH, OUT
// plain INPUT):
//   1. OUT gets the profile's bias (plain INPUT or the weak pull-up); it is
//      never an output;
//   2. warm-up: nothing is sampled, so start-up and bias-change edges are
//      never evidence;
//   3. observe: one sample per sampleIntervalMs. A level that holds for
//      stableMs ends the observation; more than BINARY_MAX_TRANSITIONS
//      changes ("toggling") or no such level within maxObserveMs
//      ("no-stable-level") is inconclusive;
//   4. floating check (bounded, once per probe): the weak pull opposing the
//      stable level for BINARY_BIAS_SETTLE_MS, one read, then the profile's
//      bias for BINARY_BIAS_SETTLE_MS, one read. Only these two reads are
//      taken under that change. A line that follows either pull is floating
//      or only weakly held ("follows-bias"): no evidence;
//   5. success with EvidenceLevel::Candidate. A rail-clamped wire passes
//      this too, which is why it is never more than a candidate.
// readSample() takes one new sample per call and never fails on a constant
// level: a static output is a valid state, and removal cannot be told from
// it. A pulse shorter than the sample interval can be missed and one
// shorter than the debounce is not shown.
//
// Guided verification ('v', then 'q', 'a', 'q'): after each mark the level
// must hold for BINARY_VERIFY_WINDOW_MS within BINARY_VERIFY_TIMEOUT_MS.
// Idle, active and return windows must read X, not X, X; the active level
// gives the polarity. Anything else stays inconclusive (Candidate). 'p'
// toggles the weak pull-up for this binding; that restarts any session
// and discards samples for BINARY_BIAS_SETTLE_MS.

static_assert(BINARY_SAMPLE_INTERVAL_MS > 0 && BINARY_SAMPLE_INTERVAL_MS < BINARY_STABLE_MS,
              "a stable window needs several samples");
static_assert(BINARY_STABLE_MS < BINARY_MAX_OBSERVE_MS, "a stable window must fit the observation budget");
static_assert(BINARY_MAX_TRANSITIONS >= 0 && BINARY_MAX_TRANSITIONS < 255, "transition limit is 8-bit");
static_assert(BINARY_VERIFY_WINDOW_MS < BINARY_VERIFY_TIMEOUT_MS, "a guided window must fit its timeout");

namespace {

bool elapsed(uint32_t now, uint32_t since, uint32_t interval) {
  return now - since >= interval;  // unsigned: correct across millis() rollover
}

const char *levelName(uint8_t level) { return level == HIGH ? "HIGH" : "LOW"; }

}  // namespace

BinaryDriver::BinaryDriver(const BinaryProfile &profile)
    : profile_(profile),
      req_{ProbeTopology::ThreeWirePowered,   // VCC / OUT / GND
           ExcitationPolicy::ProtectedActive,  // engine powers VCC/GND
           0,                                  // powerStabilizeMs: the driver's warm-up follows its bias
           profile.sampleIntervalMs,           // minAcquisitionIntervalMs
           profile.sampleIntervalMs,           // sampleIntervalMs
           // operationTimeoutMs: warm-up, the whole observation and the
           // floating check, plus headroom for slow loop passes
           static_cast<uint32_t>(profile.warmUpMs + profile.maxObserveMs + 2 * BINARY_BIAS_SETTLE_MS + 500),
           SupplyProfile::GpioLogic,  // supply: VCC/GND from GPIOs
           true,                      // openDrainSignals: OUT is never driven at all
           {nullptr, nullptr},        // signalLabel
           true} {}                   // dataIsOutput: shown as OUT, no engine pull-up

const char *BinaryDriver::getName() const { return "Binary Digital Sensor"; }

DriverOperationStatus BinaryDriver::getOperationStatus() const { return status_; }

const DriverRequirements &BinaryDriver::getRequirements() const { return req_; }

const char *BinaryDriver::getFailureReason() const { return failureReason_; }

EvidenceLevel BinaryDriver::evidence() const { return evidence_; }

bool BinaryDriver::probe(const ProbeContext &context) {
  if (context.topology != ProbeTopology::ThreeWirePowered) return fail("topology-mismatch");
  const ThreeWirePins &p = context.powered;
  if (status_ == DriverOperationStatus::Pending) {
    if (p.vcc != pins_.vcc || p.data != pins_.data || p.gnd != pins_.gnd) return fail("contract-violation");
  } else {
    // A new binding: everything learnt on the previous one is void.
    releaseLine();
    clearSession();
    pins_ = p;
    bound_ = false;
    resync_ = false;
    evidence_ = EvidenceLevel::None;
    status_ = DriverOperationStatus::Pending;
    failureReason_ = "";
    pullUp_ = profile_.bias == BinaryBias::WeakPullUp;
    setLine(biasMode());
    step_ = Step::WarmUp;
    stepSinceMs_ = millis();
  }
  return advanceProbe(millis());
}

bool BinaryDriver::advanceProbe(uint32_t now) {
  switch (step_) {
    case Step::WarmUp:
      if (!elapsed(now, stepSinceMs_, profile_.warmUpMs)) return false;
      step_ = Step::Observe;
      stepSinceMs_ = now;
      sampled_ = false;
      transitions_ = 0;
      [[fallthrough]];

    case Step::Observe: {
      if (sampled_ && !elapsed(now, lastSampleMs_, profile_.sampleIntervalMs)) return false;
      const uint8_t level = readOut();
      lastSampleMs_ = now;
      if (!sampled_ || level != raw_) {
        if (sampled_ && ++transitions_ > BINARY_MAX_TRANSITIONS) return fail("toggling");
        sampled_ = true;
        raw_ = level;
        rawSinceMs_ = now;
      }
      if (elapsed(now, rawSinceMs_, profile_.stableMs)) {
        shown_ = raw_;
        setLine(raw_ == HIGH ? INPUT_PULLDOWN : INPUT_PULLUP);  // the opposing weak pull
        step_ = Step::BiasCheck;
        stepSinceMs_ = now;
        return false;
      }
      if (elapsed(now, stepSinceMs_, profile_.maxObserveMs)) return fail("no-stable-level");
      return false;
    }

    case Step::BiasCheck:
      if (!elapsed(now, stepSinceMs_, BINARY_BIAS_SETTLE_MS)) return false;
      if (readOut() != shown_) return fail("follows-bias");
      setLine(biasMode());
      step_ = Step::BiasRestore;
      stepSinceMs_ = now;
      return false;

    case Step::BiasRestore:
      if (!elapsed(now, stepSinceMs_, BINARY_BIAS_SETTLE_MS)) return false;
      if (readOut() != shown_) return fail("follows-bias");
      break;

    case Step::Idle:
      return fail("contract-violation");
  }

  // Monitoring starts from the candidate level.
  raw_ = shown_;
  rawSinceMs_ = now;
  edges_ = 0;
  bound_ = true;
  evidence_ = EvidenceLevel::Candidate;
  step_ = Step::Idle;
  status_ = DriverOperationStatus::Succeeded;
  return true;
}

bool BinaryDriver::readSample(char *outLine1, char *outLine2, size_t maxLen) {
  if (outLine1 == nullptr || outLine2 == nullptr || maxLen == 0) {
    return fail("invalid-buffer");  // nothing dereferenced
  }
  outLine1[0] = '\0';
  outLine2[0] = '\0';
  if (status_ == DriverOperationStatus::Pending) return fail("contract-violation");
  if (!bound_) return fail("not-probed");

  const uint32_t now = millis();
  const uint8_t level = readOut();
  if (resync_) {
    // After 'p': samples are discarded while the line settles, then the
    // level is re-anchored. The change itself is never an edge.
    if (elapsed(now, resyncSinceMs_, BINARY_BIAS_SETTLE_MS)) {
      raw_ = shown_ = level;
      rawSinceMs_ = now;
      resync_ = false;
    }
  } else {
    if (level != raw_) {
      raw_ = level;
      rawSinceMs_ = now;
      if (edges_ != UINT32_MAX) edges_++;
    }
    if (raw_ != shown_ && elapsed(now, rawSinceMs_, profile_.debounceMs)) shown_ = raw_;
  }
  advanceGuided(now);
  formatLines(outLine1, outLine2, maxLen, now);
  status_ = DriverOperationStatus::Succeeded;
  return true;
}

void BinaryDriver::reset() {
  releaseLine();
  clearSession();
  status_ = DriverOperationStatus::Idle;
  failureReason_ = "";
  step_ = Step::Idle;
  pins_ = ThreeWirePins{};
  bound_ = false;
  pullUp_ = false;
  resync_ = false;
  evidence_ = EvidenceLevel::None;
  sampled_ = false;
  transitions_ = 0;
  edges_ = 0;
}

bool BinaryDriver::handleUserCommand(char command) {
  if (!bound_) return false;
  const uint32_t now = millis();
  switch (command) {
    case 'v':
    case 'V':
      if (guided_ != Guided::Off) {  // cancel
        guided_ = Guided::Off;
        return true;
      }
      clearSession();  // a new session voids the previous result
      evidence_ = EvidenceLevel::Candidate;
      guided_ = Guided::AwaitIdle;
      return true;
    case 'q':
    case 'Q':
      if (guided_ == Guided::AwaitIdle) {
        startWindow(Guided::CollectIdle, now);
      } else if (guided_ == Guided::AwaitReturn) {
        startWindow(Guided::CollectReturn, now);
      } else {
        return false;
      }
      return true;
    case 'a':
    case 'A':
      if (guided_ != Guided::AwaitActive) return false;
      startWindow(Guided::CollectActive, now);
      return true;
    case 'p':
    case 'P':
      // A deliberate bias change, never a blind one: any session and
      // verification restart, and samples wait for the line to settle.
      pullUp_ = !pullUp_;
      setLine(biasMode());
      clearSession();
      evidence_ = EvidenceLevel::Candidate;
      resync_ = true;
      resyncSinceMs_ = now;
      showMessage(pullUp_ ? "Weak pull-up: ON" : "Weak pull-up: OFF", now);
      return true;
    default:
      return false;
  }
}

bool BinaryDriver::formatDiagnostics(char *buf, size_t len) const {
  if (buf == nullptr || len == 0) return false;
  buf[0] = '\0';
  if (!bound_) return false;
  const BinaryPolarity p = polarity();
  snprintf(buf, len, "OUT raw=%u (for %lu ms) shown=%u edges=%lu bias=%s polarity=%s%s",
           raw_ == HIGH ? 1u : 0u, static_cast<unsigned long>(millis() - rawSinceMs_), shown_ == HIGH ? 1u : 0u,
           static_cast<unsigned long>(edges_), pullUp_ ? "pull-up" : "none",
           p == BinaryPolarity::ActiveLow    ? "active-low"
           : p == BinaryPolarity::ActiveHigh ? "active-high"
                                             : "unknown",
           verified_ != BinaryPolarity::Unknown ? " (user-verified)"
           : p != BinaryPolarity::Unknown       ? " (profile)"
                                                : "");
  return true;
}

// ---- Pins ------------------------------------------------------------------

uint8_t BinaryDriver::readOut() const { return digitalRead(pins_.data) == HIGH ? HIGH : LOW; }

// OUT is only ever an input: plain, or with one weak pull.
void BinaryDriver::setLine(uint8_t mode) {
  pinMode(pins_.data, mode);
  lineHeld_ = mode != INPUT;
}

uint8_t BinaryDriver::biasMode() const { return pullUp_ ? INPUT_PULLUP : INPUT; }

// Only ever makes OUT safer: a pull this driver left is removed (plain
// INPUT). A line it never touched stays untouched.
void BinaryDriver::releaseLine() {
  if (lineHeld_) pinMode(pins_.data, INPUT);
  lineHeld_ = false;
}

// ---- Guided verification ------------------------------------------------------

void BinaryDriver::startWindow(Guided collect, uint32_t now) {
  guided_ = collect;
  markMs_ = now;
  windowLevel_ = raw_;
  windowSinceMs_ = now;
  message_ = "";
}

// One step per sample: a window completes once the level has held for
// BINARY_VERIFY_WINDOW_MS since the mark or its last change.
void BinaryDriver::advanceGuided(uint32_t now) {
  if (guided_ != Guided::CollectIdle && guided_ != Guided::CollectActive && guided_ != Guided::CollectReturn) {
    return;
  }
  if (raw_ != windowLevel_) {
    windowLevel_ = raw_;
    windowSinceMs_ = now;
  }
  if (!elapsed(now, windowSinceMs_, BINARY_VERIFY_WINDOW_MS)) {
    if (elapsed(now, markMs_, BINARY_VERIFY_TIMEOUT_MS)) {  // never settled: redo this step
      guided_ = guided_ == Guided::CollectIdle     ? Guided::AwaitIdle
                : guided_ == Guided::CollectActive ? Guided::AwaitActive
                                                   : Guided::AwaitReturn;
      showMessage("Verify: unsteady", now);
    }
    return;
  }
  switch (guided_) {
    case Guided::CollectIdle:
      idleLevel_ = windowLevel_;
      guided_ = Guided::AwaitActive;
      break;
    case Guided::CollectActive:
      if (windowLevel_ == idleLevel_) {  // no response: inconclusive
        guided_ = Guided::Off;
        showMessage("Verify: no change", now);
        break;
      }
      activeLevel_ = windowLevel_;
      guided_ = Guided::AwaitReturn;
      break;
    case Guided::CollectReturn:
      guided_ = Guided::Off;
      if (windowLevel_ != idleLevel_) {  // not repeatable: inconclusive
        showMessage("Verify: no return", now);
        break;
      }
      verified_ = activeLevel_ == LOW ? BinaryPolarity::ActiveLow : BinaryPolarity::ActiveHigh;
      evidence_ = EvidenceLevel::UserVerified;
      showMessage(verified_ == BinaryPolarity::ActiveLow ? "Verified active-LOW" : "Verified active-HIGH", now);
      break;
    default:
      break;
  }
}

void BinaryDriver::showMessage(const char *text, uint32_t now) {
  message_ = text;
  messageSinceMs_ = now;
}

void BinaryDriver::clearSession() {
  guided_ = Guided::Off;
  verified_ = BinaryPolarity::Unknown;
  message_ = "";
}

BinaryPolarity BinaryDriver::polarity() const {
  return verified_ != BinaryPolarity::Unknown ? verified_ : profile_.polarity;
}

// Line 1 keeps the raw level beside its interpretation: "State: ACTIVE (0)"
// with a known polarity, else "Level: HIGH (1)". Line 2: the type, or the
// guided prompt/result.
void BinaryDriver::formatLines(char *line1, char *line2, size_t maxLen, uint32_t now) const {
  const unsigned bit = shown_ == HIGH ? 1u : 0u;
  const BinaryPolarity p = polarity();
  if (p == BinaryPolarity::Unknown) {
    snprintf(line1, maxLen, "Level: %s (%u)", levelName(shown_), bit);
  } else {
    const bool active = (p == BinaryPolarity::ActiveHigh) == (shown_ == HIGH);
    snprintf(line1, maxLen, "State: %s (%u)", active ? "ACTIVE" : "IDLE", bit);
  }

  const char *text = "Type: Digital IO";
  if (message_[0] != '\0' && !elapsed(now, messageSinceMs_, BINARY_MESSAGE_MS)) {
    text = message_;
  } else {
    switch (guided_) {
      case Guided::AwaitIdle: text = "No stimulus, send q"; break;
      case Guided::CollectIdle: text = "Reading idle..."; break;
      case Guided::AwaitActive: text = "Stimulus on, send a"; break;
      case Guided::CollectActive: text = "Reading active..."; break;
      case Guided::AwaitReturn: text = "Stimulus off, send q"; break;
      case Guided::CollectReturn: text = "Reading return..."; break;
      case Guided::Off: break;
    }
  }
  snprintf(line2, maxLen, "%s", text);
}

bool BinaryDriver::fail(const char *reason) {
  releaseLine();
  status_ = DriverOperationStatus::Failed;
  failureReason_ = reason;
  step_ = Step::Idle;
  bound_ = false;
  evidence_ = EvidenceLevel::None;
  return false;
}
