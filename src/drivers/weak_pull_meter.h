// One weak-pull divider measurement of a node pair in one orientation
// (sense, sink), shared by PassiveDriver and the engine's pull-up
// classification. The only excitation is the internal pull-up on the sense
// node (passive_adc.h) and an open-drain LOW on the sink node: no pin is
// ever driven HIGH and no other pin is touched.
//   1. both pins plain INPUT;
//   2. sense: ADC channel + internal pull-up;
//   3. reference window with the sink released: the pull-up must lift the
//      sense node to PASSIVE_PULLUP_VERIFY_MIN_MV, which verifies it is
//      effective while sampled ("pullup-unverified" otherwise);
//   4. sink: latch LOW, then OUTPUT_OPEN_DRAIN (it cannot drive HIGH);
//   5. divider window of PASSIVE_WINDOW_SAMPLES readings;
//   6. release: sink, then sense, back to plain INPUT.
// Cooperative: advance() waits by returning Pending and does at most one
// ADC read per call. A failed read is never taken as a 0 mV reading.
#pragma once

#include <stdint.h>

// Approximate divider estimate R ~ Rpullup * V / (Vsupply - V), treating
// the open-drain LOW sink as 0 V and the pull-up as its nominal value.
// False outside [PASSIVE_CONTACT_MAX_MV, PASSIVE_OPEN_MIN_MV], where the
// reading is a bound (contact-like / open-like), not a value.
bool passiveEstimateOhms(uint16_t senseMv, uint32_t &ohms);

class WeakPullMeter {
 public:
  enum class Result : uint8_t { Pending, Done, Failed };

  // Steps 1-2; settleMs is the wait after each excitation change. False
  // (pins already released) if the ADC could not be set up.
  bool start(uint8_t sensePin, uint8_t sinkPin, uint32_t settleMs);

  // Steps 3-6. Done and Failed both leave the pins released.
  Result advance();

  // Sink, then sense, back to plain INPUT. Idempotent; never energises and
  // touches nothing when no measurement is active.
  void release();

  uint16_t meanMv() const { return meanMv_; }      // divider window mean, after Done
  uint16_t spreadMv() const { return spreadMv_; }  // max - min of that window

  // After Failed: "adc-unavailable", "adc-config-failed", "adc-read-failed",
  // "pullup-unverified" or "not-started"; "" otherwise.
  const char *failureReason() const { return failureReason_; }

 private:
  enum class Step : uint8_t { Idle, Reference, Divider };

  Result fail(const char *reason);
  void startWindow();

  uint8_t sensePin_ = 0;
  uint8_t sinkPin_ = 0;
  bool senseActive_ = false;  // ADC + pull-up configured on sensePin_
  bool sinkActive_ = false;   // open-drain LOW on sinkPin_
  Step step_ = Step::Idle;
  uint32_t settleMs_ = 0;
  uint32_t stepSinceMs_ = 0;
  uint32_t waitMs_ = 0;
  uint8_t samples_ = 0;
  uint32_t sumMv_ = 0;
  uint16_t minMv_ = 0;
  uint16_t maxMv_ = 0;
  uint16_t meanMv_ = 0;
  uint16_t spreadMv_ = 0;
  const char *failureReason_ = "";
};
