// Binary digital sensor driver: ThreeWirePowered on the GpioLogic profile,
// with DATA as the module's output (OUT, only ever read: never OUTPUT, DAC
// or PWM). Power is the engine's job.
//
// Discovery reports at most a Candidate: a driven, stable logic level is
// plausible evidence of a three-terminal digital-output module, never
// proof of a valid input voltage, an active driver, correct supply
// polarity, the right OUT node or a model. A guided test, in which the user
// applies and removes the real stimulus, can raise it to UserVerified for
// this binding and sets the active polarity. Signals shorter than the
// sampling policy can be missed: there is no edge capture.
#pragma once

#include "../../config.h"
#include "../../sensor_driver.h"

enum class BinaryBias : uint8_t {
  None,        // plain INPUT (default)
  WeakPullUp,  // internal pull-up to 3.3 V, e.g. an open-collector output without its own
};

enum class BinaryPolarity : uint8_t { Unknown, ActiveLow, ActiveHigh };

// One input profile. Nothing here is a datasheet figure.
struct BinaryProfile {
  uint32_t warmUpMs;          // OUT bias applied -> first counted sample
  uint32_t sampleIntervalMs;  // between samples, probing and monitoring
  uint32_t stableMs;          // a candidate level must hold this long
  uint32_t debounceMs;        // monitoring: a new level is shown after holding this long
  uint32_t maxObserveMs;      // observation budget after warm-up
  BinaryBias bias;
  BinaryPolarity polarity;    // Unknown: the raw level is shown until a guided test
};

// The discovery default (config.h): plain INPUT, unknown polarity.
constexpr BinaryProfile kBinaryGenericProfile = {
    BINARY_WARMUP_MS,     BINARY_SAMPLE_INTERVAL_MS, BINARY_STABLE_MS, BINARY_DEBOUNCE_MS,
    BINARY_MAX_OBSERVE_MS, BinaryBias::None,          BinaryPolarity::Unknown,
};

class BinaryDriver final : public SensorDriver {
 public:
  explicit BinaryDriver(const BinaryProfile &profile = kBinaryGenericProfile);

  const char *getName() const override;
  bool probe(const ProbeContext &context) override;
  bool readSample(char *outLine1, char *outLine2, size_t maxLen) override;
  void reset() override;
  DriverOperationStatus getOperationStatus() const override;
  const DriverRequirements &getRequirements() const override;
  const char *getFailureReason() const override;
  // Guided verification: 'v' start/cancel, 'q' mark idle (stimulus
  // removed), 'a' mark active (stimulus applied). 'p' toggles the weak
  // pull-up for this binding (e.g. an open collector without its own).
  bool handleUserCommand(char command) override;
  // "OUT raw=.. shown=.. ..." from cached samples only.
  bool formatDiagnostics(char *buf, size_t len) const override;
  EvidenceLevel evidence() const override;

 private:
  enum class Step : uint8_t { Idle, WarmUp, Observe, BiasCheck, BiasRestore };
  enum class Guided : uint8_t { Off, AwaitIdle, CollectIdle, AwaitActive, CollectActive, AwaitReturn, CollectReturn };

  bool advanceProbe(uint32_t now);
  uint8_t readOut() const;
  void setLine(uint8_t mode);
  uint8_t biasMode() const;
  void releaseLine();
  void startWindow(Guided collect, uint32_t now);
  void advanceGuided(uint32_t now);
  void showMessage(const char *text, uint32_t now);
  void clearSession();
  BinaryPolarity polarity() const;
  void formatLines(char *line1, char *line2, size_t maxLen, uint32_t now) const;
  bool fail(const char *reason);

  BinaryProfile profile_;
  DriverRequirements req_;

  // Operation and binding.
  DriverOperationStatus status_ = DriverOperationStatus::Idle;
  const char *failureReason_ = "";
  Step step_ = Step::Idle;
  ThreeWirePins pins_ = {};
  bool bound_ = false;     // a candidate level from a successful probe() on pins_
  bool lineHeld_ = false;  // this driver left a pull on OUT
  bool pullUp_ = false;    // this binding's bias: the profile's, or toggled with 'p'
  bool resync_ = false;    // bias just changed: samples discarded until it settles
  uint32_t resyncSinceMs_ = 0;
  EvidenceLevel evidence_ = EvidenceLevel::None;

  // Samples: probing and monitoring.
  uint32_t stepSinceMs_ = 0;
  uint32_t lastSampleMs_ = 0;
  bool sampled_ = false;     // this observation has a first sample
  uint8_t transitions_ = 0;  // raw changes in this observation
  uint8_t raw_ = 0;          // latest sample (LOW/HIGH)
  uint32_t rawSinceMs_ = 0;  // raw_ unchanged since
  uint8_t shown_ = 0;        // probing: the candidate level; monitoring: the debounced level
  uint32_t edges_ = 0;       // raw changes while monitoring (diagnostics)

  // Guided verification, bound to this binding (reset() and probe() clear it).
  Guided guided_ = Guided::Off;
  uint32_t markMs_ = 0;
  uint8_t windowLevel_ = 0;
  uint32_t windowSinceMs_ = 0;
  uint8_t idleLevel_ = 0;
  uint8_t activeLevel_ = 0;
  BinaryPolarity verified_ = BinaryPolarity::Unknown;
  const char *message_ = "";
  uint32_t messageSinceMs_ = 0;
};
