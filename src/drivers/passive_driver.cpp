#include "passive_driver.h"

#include <Arduino.h>
#include <stdio.h>

#include "../../config.h"
#include "passive_adc.h"

// Measurement, per orientation of the probed pair (sense, sink):
//   1. both terminals plain INPUT; the two unused nodes are never touched;
//   2. sense: ADC channel + internal pull-up (adapter);
//   3. reference window with the sink still released: the pull-up must lift
//      the node near the rail, which verifies it is effective while sampled;
//   4. sink: latch LOW, then OUTPUT_OPEN_DRAIN (cannot drive HIGH, ever);
//   5. divider window; 6. release sink and sense back to plain INPUT.
// A probe runs both orientations and requires symmetric, stable results.
// Every wait is cooperative (Pending); each call does at most one ADC read.

static_assert(PASSIVE_CONTACT_MAX_MV < PASSIVE_OPEN_MIN_MV, "contact and open bands overlap");
static_assert(PASSIVE_OPEN_MIN_MV < PASSIVE_SUPPLY_MV, "the divider estimate needs V < Vsupply");
static_assert(PASSIVE_PULLUP_VERIFY_MIN_MV < PASSIVE_SUPPLY_MV, "pull-up check above the rail");
static_assert(PASSIVE_LDR_MIN_SEPARATION_MV > 0, "LDR calibration needs a positive span");
static_assert(PASSIVE_DEBOUNCE_SAMPLES >= 1 && PASSIVE_WINDOW_SAMPLES >= 1 &&
                  PASSIVE_REFERENCE_SAMPLES >= 1 && PASSIVE_LDR_WINDOW_SAMPLES >= 1,
              "windows need samples");

namespace {

const DriverRequirements kRequirements = {
    ProbeTopology::TwoTerminal,     // terminal pair + unused node
    ExcitationPolicy::WeakPullOnly,  // the engine never powers anything for it
    0,                               // powerStabilizeMs: nothing is powered
    PASSIVE_SAMPLE_PERIOD_MS,        // minAcquisitionIntervalMs
    PASSIVE_SAMPLE_PERIOD_MS,        // sampleIntervalMs
    PASSIVE_OPERATION_TIMEOUT_MS,    // operationTimeoutMs
    0,                               // backendCapabilities
    {nullptr, nullptr},              // signalLabel
};

const size_t kTextLen = 24;

uint16_t absDiff(uint16_t a, uint16_t b) { return a > b ? a - b : b - a; }

// A value inside the useful window, a bound outside it; always "R~".
void formatResistance(char *out, size_t len, uint16_t mv) {
  uint32_t ohms = 0;
  const char *prefix = "R~ ";
  if (mv <= PASSIVE_CONTACT_MAX_MV) {
    passiveEstimateOhms(PASSIVE_CONTACT_MAX_MV, ohms);
    prefix = "R~ < ";
  } else if (mv >= PASSIVE_OPEN_MIN_MV) {
    passiveEstimateOhms(PASSIVE_OPEN_MIN_MV, ohms);
    prefix = "R~ > ";
  } else {
    passiveEstimateOhms(mv, ohms);
  }
  if (ohms < 1000UL) {
    snprintf(out, len, "%s%lu Ohm", prefix, static_cast<unsigned long>(ohms));
  } else if (ohms < 100000UL) {
    snprintf(out, len, "%s%lu.%lu kOhm", prefix, static_cast<unsigned long>(ohms / 1000UL),
             static_cast<unsigned long>((ohms % 1000UL) / 100UL));
  } else {
    snprintf(out, len, "%s%lu kOhm", prefix, static_cast<unsigned long>((ohms + 500UL) / 1000UL));
  }
}

// Relative conduction, 0% (open) to 100% (contact). Not a physical unit.
unsigned signalPercent(uint16_t mv) {
  const uint32_t v = mv < PASSIVE_SUPPLY_MV ? mv : PASSIVE_SUPPLY_MV;
  return static_cast<unsigned>((PASSIVE_SUPPLY_MV - v) * 100UL / PASSIVE_SUPPLY_MV);
}

}  // namespace

bool passiveEstimateOhms(uint16_t senseMv, uint32_t &ohms) {
  if (senseMv < PASSIVE_CONTACT_MAX_MV || senseMv > PASSIVE_OPEN_MIN_MV ||
      senseMv >= PASSIVE_SUPPLY_MV) {
    return false;
  }
  const uint32_t denominator = PASSIVE_SUPPLY_MV - senseMv;  // > 0 by the checks above
  ohms = static_cast<uint32_t>(static_cast<uint64_t>(PASSIVE_PULLUP_NOMINAL_OHMS) * senseMv /
                               denominator);
  return true;
}

const char *PassiveDriver::getName() const {
  switch (kind_) {
    case Kind::Contact: return "Contact / Low R";
    case Kind::Switch: return "Push Button / Switch";
    case Kind::Resistive: return "Resistive Component";
    case Kind::Variable: return "Variable Resistance";
    case Kind::Ldr: return "Photoresistor (LDR)";  // only after guided calibration
    case Kind::Unknown: break;
  }
  return "Passive Component";
}

DriverOperationStatus PassiveDriver::getOperationStatus() const { return status_; }

const DriverRequirements &PassiveDriver::getRequirements() const { return kRequirements; }

const char *PassiveDriver::getFailureReason() const { return failureReason_; }

bool PassiveDriver::failureIsMeasurementError() const { return measurementError_; }

bool PassiveDriver::probe(const ProbeContext &context) {
  if (context.topology != ProbeTopology::TwoTerminal) return fail("topology-mismatch", false);
  if (status_ == DriverOperationStatus::Pending) {
    if (operation_ != Operation::Probe || context.pair.first != first_ ||
        context.pair.second != second_) {
      return fail("contract-violation", false);
    }
  } else {
    first_ = context.pair.first;  // context.pair.unused[] is deliberately ignored
    second_ = context.pair.second;
    bound_ = false;
    clearClassification();  // new session: calibration and history invalidated
    operation_ = Operation::Probe;
    status_ = DriverOperationStatus::Pending;
    failureReason_ = "";
    measurementError_ = false;
    if (!startOrientation(false)) return false;
  }
  if (advance() != Advance::OrientationDone) return false;
  if (!reversed_) {
    startOrientation(true);  // second orientation: Pending (or already failed)
    return false;
  }
  return finishProbe();
}

bool PassiveDriver::readSample(char *outLine1, char *outLine2, size_t maxLen) {
  if (outLine1 == nullptr || outLine2 == nullptr || maxLen == 0) {
    return fail("invalid-buffer", false);  // nothing dereferenced
  }
  outLine1[0] = '\0';
  outLine2[0] = '\0';
  if (status_ == DriverOperationStatus::Pending) {
    if (operation_ != Operation::Sample) return fail("contract-violation", false);
  } else {
    if (!bound_) return fail("not-probed", false);
    operation_ = Operation::Sample;
    status_ = DriverOperationStatus::Pending;
    failureReason_ = "";
    measurementError_ = false;
    if (!startOrientation(false)) return false;
  }
  if (advance() != Advance::OrientationDone) return false;
  return finishSample(outLine1, outLine2, maxLen);
}

void PassiveDriver::reset() {
  releaseExcitation();
  operation_ = Operation::None;
  status_ = DriverOperationStatus::Idle;
  failureReason_ = "";
  measurementError_ = false;
  bound_ = false;
  first_ = second_ = sensePin_ = sinkPin_ = 0;
  reversed_ = false;
  step_ = Step::Idle;
  samples_ = 0;
  clearClassification();
}

bool PassiveDriver::handleUserCommand(char command) {
  const bool resistive = kind_ == Kind::Resistive || kind_ == Kind::Variable || kind_ == Kind::Ldr;
  switch (command) {
    case 'l':
    case 'L':
      if (!resistive) return false;
      if (guided_ != Guided::Off) {  // cancel
        guided_ = Guided::Off;
        return true;
      }
      if (kind_ == Kind::Ldr) kind_ = Kind::Variable;  // old calibration void until redone
      guided_ = Guided::AwaitDark;
      messageLeft_ = 0;
      return true;
    case 'd':
    case 'D':
      if (guided_ != Guided::AwaitDark) return false;
      beginCalibrationWindow(Guided::CollectDark);
      return true;
    case 'b':
    case 'B':
      if (guided_ != Guided::AwaitBright) return false;
      beginCalibrationWindow(Guided::CollectBright);
      return true;
    default:
      return false;
  }
}

// ---- Measurement ---------------------------------------------------------------

bool PassiveDriver::startOrientation(bool reversed) {
  reversed_ = reversed;
  sensePin_ = reversed ? second_ : first_;
  sinkPin_ = reversed ? first_ : second_;
  pinMode(first_, INPUT);  // both terminals high impedance before excitation
  pinMode(second_, INPUT);
  senseActive_ = true;  // so every failure path releases it
  const PassiveAdcStatus begun = passiveAdcBeginSense(sensePin_);
  if (begun != PassiveAdcStatus::Ok) {
    return fail(begun == PassiveAdcStatus::Unavailable ? "adc-unavailable" : "adc-config-failed",
                true);
  }
  step_ = Step::Reference;
  samples_ = 0;
  sumMv_ = 0;
  minMv_ = 0xFFFF;
  maxMv_ = 0;
  stepSinceMs_ = millis();
  waitMs_ = PASSIVE_SETTLE_MS;
  return true;
}

PassiveDriver::Advance PassiveDriver::advance() {
  const uint32_t now = millis();
  if (now - stepSinceMs_ < waitMs_) return Advance::Pending;  // rollover-safe
  stepSinceMs_ = now;
  waitMs_ = PASSIVE_SAMPLE_INTERVAL_MS;

  uint16_t mv = 0;
  if (passiveAdcReadMv(sensePin_, mv) != PassiveAdcStatus::Ok) {
    fail("adc-read-failed", true);  // never confused with a valid 0 mV
    return Advance::Failed;
  }

  if (step_ == Step::Reference) {
    // Only the pull-up acts on this node now. If it does not sit near the
    // rail, the pull-up is not effective while sampled (or something
    // external holds the node), so no divider reading can be trusted.
    if (mv < PASSIVE_PULLUP_VERIFY_MIN_MV) {
      fail("pullup-unverified", true);
      return Advance::Failed;
    }
    if (++samples_ < PASSIVE_REFERENCE_SAMPLES) return Advance::Pending;
    digitalWrite(sinkPin_, LOW);           // latch LOW first...
    pinMode(sinkPin_, OUTPUT_OPEN_DRAIN);  // ...open-drain: sinks only, never HIGH
    sinkActive_ = true;
    step_ = Step::Divider;
    samples_ = 0;
    sumMv_ = 0;
    minMv_ = 0xFFFF;
    maxMv_ = 0;
    waitMs_ = PASSIVE_SETTLE_MS;
    return Advance::Pending;
  }

  sumMv_ += mv;
  if (mv < minMv_) minMv_ = mv;
  if (mv > maxMv_) maxMv_ = mv;
  if (++samples_ < PASSIVE_WINDOW_SAMPLES) return Advance::Pending;
  const int slot = reversed_ ? 1 : 0;
  resultMv_[slot] = static_cast<uint16_t>(sumMv_ / samples_);
  spreadMv_[slot] = static_cast<uint16_t>(maxMv_ - minMv_);
  releaseExcitation();
  step_ = Step::Idle;
  return Advance::OrientationDone;
}

// Both orientations measured: accept only a stable, symmetric, conducting
// pair. Asymmetry (junctions, unpowered active modules, odd networks) or
// noise is not interpreted.
bool PassiveDriver::finishProbe() {
  if (spreadMv_[0] > PASSIVE_STABLE_SPREAD_MV || spreadMv_[1] > PASSIVE_STABLE_SPREAD_MV) {
    return fail("unstable", false);
  }
  const uint16_t mean = static_cast<uint16_t>((static_cast<uint32_t>(resultMv_[0]) + resultMv_[1]) / 2);
  const uint32_t pctTolerance = static_cast<uint32_t>(mean) * PASSIVE_SYMMETRY_PCT / 100;
  const uint32_t tolerance =
      pctTolerance > PASSIVE_SYMMETRY_MIN_DELTA_MV ? pctTolerance : PASSIVE_SYMMETRY_MIN_DELTA_MV;
  if (absDiff(resultMv_[0], resultMv_[1]) > tolerance) return fail("asymmetric", false);
  if (mean >= PASSIVE_OPEN_MIN_MV) return fail("open-circuit", false);  // nothing (or > ~130k)

  kind_ = mean <= PASSIVE_CONTACT_MAX_MV ? Kind::Contact : Kind::Resistive;
  closed_ = kind_ == Kind::Contact;
  baselineMv_ = mean;
  filteredMv_ = mean;
  bound_ = true;
  operation_ = Operation::None;
  status_ = DriverOperationStatus::Succeeded;
  return true;
}

bool PassiveDriver::finishSample(char *line1, char *line2, size_t maxLen) {
  const uint16_t mv = resultMv_[0];
  if (kind_ == Kind::Contact || kind_ == Kind::Switch) {
    updateContact(mv);  // OPEN is a valid state, never a failure
  } else if (!updateResistive(mv)) {
    return fail("open-circuit", false);  // open-like past the grace period
  }
  formatLines(line1, line2, maxLen, mv);
  if (messageLeft_ > 0) messageLeft_--;
  operation_ = Operation::None;
  status_ = DriverOperationStatus::Succeeded;
  return true;
}

// Debounced OPEN/CLOSED with hysteresis: only clearly closed (contact band)
// or clearly open (open band) readings count toward a transition. One
// transition makes it "switch-like"; a constant contact stays a contact.
void PassiveDriver::updateContact(uint16_t mv) {
  const bool towardOpen = closed_ && mv >= PASSIVE_OPEN_MIN_MV;
  const bool towardClosed = !closed_ && mv <= PASSIVE_CONTACT_MAX_MV;
  if (!towardOpen && !towardClosed) {
    debounce_ = 0;
    return;
  }
  if (++debounce_ < PASSIVE_DEBOUNCE_SAMPLES) return;
  closed_ = !closed_;
  debounce_ = 0;
  kind_ = Kind::Switch;  // observed switch-like behaviour, not proof of a button
}

// False once an open-like reading outlasts the grace period (very high R or
// removed: indistinguishable here, so the engine's retry policy decides).
bool PassiveDriver::updateResistive(uint16_t mv) {
  if (guided_ == Guided::CollectDark || guided_ == Guided::CollectBright) collectCalibration(mv);
  openLike_ = mv >= PASSIVE_OPEN_MIN_MV;
  if (openLike_ && guided_ == Guided::Off) {
    if (++openRun_ > PASSIVE_OPEN_GRACE_SAMPLES) return false;
  } else {
    openRun_ = 0;
  }

  const int32_t delta = static_cast<int32_t>(mv) - static_cast<int32_t>(filteredMv_);
  filteredMv_ = static_cast<uint16_t>(filteredMv_ + delta / (1 << PASSIVE_FILTER_SHIFT));
  if (kind_ == Kind::Resistive && !openLike_) {
    if (absDiff(filteredMv_, baselineMv_) >= PASSIVE_VARIATION_MIN_MV) {
      if (++variationRun_ >= PASSIVE_VARIATION_CONFIRM_SAMPLES) kind_ = Kind::Variable;
    } else {
      variationRun_ = 0;
    }
  }
  return true;
}

void PassiveDriver::beginCalibrationWindow(Guided collect) {
  guided_ = collect;
  calSumMv_ = 0;
  calMinMv_ = 0xFFFF;
  calMaxMv_ = 0;
  calSamples_ = 0;
  messageLeft_ = 0;
}

void PassiveDriver::showMessage(const char *text) {
  message_ = text;
  messageLeft_ = PASSIVE_MESSAGE_SAMPLES;
}

// The LDR name and "Light: %" exist only after the user covered and lit the
// part and the two references were steady and well separated.
void PassiveDriver::collectCalibration(uint16_t mv) {
  calSumMv_ += mv;
  if (mv < calMinMv_) calMinMv_ = mv;
  if (mv > calMaxMv_) calMaxMv_ = mv;
  if (++calSamples_ < PASSIVE_LDR_WINDOW_SAMPLES) return;

  const uint16_t average = static_cast<uint16_t>(calSumMv_ / calSamples_);
  const bool steady = calMaxMv_ - calMinMv_ <= PASSIVE_LDR_MAX_WINDOW_SPREAD_MV;
  if (guided_ == Guided::CollectDark) {
    if (steady) {
      darkMv_ = average;
      guided_ = Guided::AwaitBright;
    } else {
      guided_ = Guided::AwaitDark;
      showMessage("Cal: unsteady, redo");
    }
    return;
  }
  if (!steady) {
    guided_ = Guided::AwaitBright;
    showMessage("Cal: unsteady, redo");
    return;
  }
  brightMv_ = average;
  guided_ = Guided::Off;
  if (darkMv_ >= brightMv_ + PASSIVE_LDR_MIN_SEPARATION_MV) {
    kind_ = Kind::Ldr;
  } else {
    showMessage("Cal failed: no change");  // no percentage is fabricated
  }
}

const char *PassiveDriver::guidedPrompt(bool firstLine) const {
  switch (guided_) {
    case Guided::AwaitDark: return firstLine ? "LDR cal: cover it" : "then send 'd'";
    case Guided::CollectDark: return firstLine ? "LDR cal: dark..." : "keep it covered";
    case Guided::AwaitBright: return firstLine ? "LDR cal: light it" : "then send 'b'";
    case Guided::CollectBright: return firstLine ? "LDR cal: bright..." : "keep it lit";
    case Guided::Off: break;
  }
  return "";
}

void PassiveDriver::formatLines(char *line1, char *line2, size_t maxLen, uint16_t mv) const {
  char resistance[kTextLen];
  formatResistance(resistance, sizeof resistance, mv);

  if (kind_ == Kind::Contact || kind_ == Kind::Switch) {
    snprintf(line1, maxLen, "State: %s", closed_ ? "CLOSED" : "OPEN");
    if (kind_ == Kind::Contact) snprintf(line2, maxLen, "%s", resistance);
    return;
  }
  if (guided_ != Guided::Off) {
    snprintf(line1, maxLen, "%s", messageLeft_ > 0 ? message_ : guidedPrompt(true));
    snprintf(line2, maxLen, "%s", guidedPrompt(false));
    return;
  }
  if (openLike_) {
    snprintf(line1, maxLen, "%s", resistance);
    snprintf(line2, maxLen, "Open or removed?");
    return;
  }
  if (kind_ == Kind::Ldr) {
    // Relative index between the user's own dark and bright references,
    // clamped to 0-100. Not lux, not an absolute light level.
    const int32_t span = static_cast<int32_t>(darkMv_) - brightMv_;  // > 0 by acceptance
    int32_t light = (static_cast<int32_t>(darkMv_) - mv) * 100 / span;
    if (light < 0) light = 0;
    if (light > 100) light = 100;
    snprintf(line1, maxLen, "Light: %ld%%", static_cast<long>(light));
    snprintf(line2, maxLen, "%s", resistance);
  } else {
    snprintf(line1, maxLen, "%s", resistance);
    if (kind_ == Kind::Variable) {
      snprintf(line2, maxLen, "Sig %u%% Possible LDR", signalPercent(mv));
    } else {
      snprintf(line2, maxLen, "Signal: %u%%", signalPercent(mv));
    }
  }
  if (messageLeft_ > 0) snprintf(line2, maxLen, "%s", message_);
}

bool PassiveDriver::fail(const char *reason, bool measurementError) {
  releaseExcitation();
  operation_ = Operation::None;
  status_ = DriverOperationStatus::Failed;
  failureReason_ = reason;
  measurementError_ = measurementError;
  step_ = Step::Idle;
  return false;
}

// Only ever makes the pair safer: the sink back to plain INPUT, the sense
// pull-up off and its pad back to a digital INPUT. Untouched pins stay so.
void PassiveDriver::releaseExcitation() {
  if (sinkActive_) {
    pinMode(sinkPin_, INPUT);
    sinkActive_ = false;
  }
  if (senseActive_) {
    passiveAdcEndSense(sensePin_);
    senseActive_ = false;
  }
}

void PassiveDriver::clearClassification() {
  kind_ = Kind::Unknown;
  closed_ = false;
  debounce_ = 0;
  baselineMv_ = 0;
  filteredMv_ = 0;
  variationRun_ = 0;
  openRun_ = 0;
  openLike_ = false;
  guided_ = Guided::Off;
  calSumMv_ = 0;
  calMinMv_ = 0;
  calMaxMv_ = 0;
  calSamples_ = 0;
  darkMv_ = 0;
  brightMv_ = 0;
  message_ = "";
  messageLeft_ = 0;
}
