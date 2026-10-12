#include "weak_pull_meter.h"

#include <Arduino.h>

#include "../../config.h"
#include "passive_adc.h"

static_assert(PASSIVE_PULLUP_VERIFY_MIN_MV < PASSIVE_SUPPLY_MV, "pull-up check above the rail");
static_assert(PASSIVE_WINDOW_SAMPLES >= 1 && PASSIVE_REFERENCE_SAMPLES >= 1, "windows need samples");

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

bool WeakPullMeter::start(uint8_t sensePin, uint8_t sinkPin, uint32_t settleMs) {
  release();
  sensePin_ = sensePin;
  sinkPin_ = sinkPin;
  settleMs_ = settleMs;
  failureReason_ = "";
  pinMode(sensePin, INPUT);  // both terminals high impedance before excitation
  pinMode(sinkPin, INPUT);
  senseActive_ = true;  // so every failure path releases it
  const PassiveAdcStatus begun = passiveAdcBeginSense(sensePin);
  if (begun != PassiveAdcStatus::Ok) {
    fail(begun == PassiveAdcStatus::Unavailable ? "adc-unavailable" : "adc-config-failed");
    return false;
  }
  step_ = Step::Reference;
  startWindow();
  stepSinceMs_ = millis();
  return true;
}

void WeakPullMeter::startWindow() {
  samples_ = 0;
  sumMv_ = 0;
  minMv_ = 0xFFFF;
  maxMv_ = 0;
  waitMs_ = settleMs_;
}

WeakPullMeter::Result WeakPullMeter::advance() {
  if (step_ == Step::Idle) return fail("not-started");
  const uint32_t now = millis();
  if (now - stepSinceMs_ < waitMs_) return Result::Pending;  // rollover-safe
  stepSinceMs_ = now;
  waitMs_ = PASSIVE_SAMPLE_INTERVAL_MS;

  uint16_t mv = 0;
  if (passiveAdcReadMv(sensePin_, mv) != PassiveAdcStatus::Ok) {
    return fail("adc-read-failed");  // never confused with a valid 0 mV
  }

  if (step_ == Step::Reference) {
    // Only the pull-up acts on this node now. If it does not sit near the
    // rail, the pull-up is not effective while sampled (or something
    // external holds the node), so no divider reading can be trusted.
    if (mv < PASSIVE_PULLUP_VERIFY_MIN_MV) return fail("pullup-unverified");
    if (++samples_ < PASSIVE_REFERENCE_SAMPLES) return Result::Pending;
    digitalWrite(sinkPin_, LOW);           // latch LOW first...
    pinMode(sinkPin_, OUTPUT_OPEN_DRAIN);  // ...open-drain: sinks only, never HIGH
    sinkActive_ = true;
    step_ = Step::Divider;
    startWindow();
    return Result::Pending;
  }

  sumMv_ += mv;
  if (mv < minMv_) minMv_ = mv;
  if (mv > maxMv_) maxMv_ = mv;
  if (++samples_ < PASSIVE_WINDOW_SAMPLES) return Result::Pending;
  meanMv_ = static_cast<uint16_t>(sumMv_ / samples_);
  spreadMv_ = static_cast<uint16_t>(maxMv_ - minMv_);
  release();
  return Result::Done;
}

WeakPullMeter::Result WeakPullMeter::fail(const char *reason) {
  release();
  failureReason_ = reason;
  return Result::Failed;
}

// Only ever makes the pair safer: the sink back to plain INPUT, the sense
// pull-up off and its pad back to a digital INPUT.
void WeakPullMeter::release() {
  if (sinkActive_) {
    pinMode(sinkPin_, INPUT);
    sinkActive_ = false;
  }
  if (senseActive_) {
    passiveAdcEndSense(sensePin_);
    senseActive_ = false;
  }
  step_ = Step::Idle;
}
