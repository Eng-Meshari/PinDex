// Passive two-terminal driver: resistive parts, contacts and switch-like
// behaviour, measured with weak internal pull-up excitation only (see
// passive_adc.h). Classification is conservative and evidence-based; it
// never proves what the part is.
#pragma once

#include "../../sensor_driver.h"

// Approximate divider estimate R ~ Rpullup * V / (Vsupply - V), treating
// the open-drain LOW sink as 0 V and the pull-up as its nominal value.
// False outside [PASSIVE_CONTACT_MAX_MV, PASSIVE_OPEN_MIN_MV], where the
// reading is a bound (contact-like / open-like), not a value.
bool passiveEstimateOhms(uint16_t senseMv, uint32_t &ohms);

class PassiveDriver final : public SensorDriver {
 public:
  const char *getName() const override;
  bool probe(const ProbeContext &context) override;
  bool readSample(char *outLine1, char *outLine2, size_t maxLen) override;
  void reset() override;
  DriverOperationStatus getOperationStatus() const override;
  const DriverRequirements &getRequirements() const override;
  const char *getFailureReason() const override;
  bool failureIsMeasurementError() const override;
  // Guided LDR mode: 'l' start/cancel, 'd' sample dark, 'b' sample bright.
  bool handleUserCommand(char command) override;

 private:
  enum class Operation : uint8_t { None, Probe, Sample };
  enum class Step : uint8_t { Idle, Reference, Divider };
  enum class Advance : uint8_t { Pending, OrientationDone, Failed };
  enum class Kind : uint8_t { Unknown, Contact, Switch, Resistive, Variable, Ldr };
  enum class Guided : uint8_t { Off, AwaitDark, CollectDark, AwaitBright, CollectBright };

  bool startOrientation(bool reversed);
  Advance advance();
  bool finishProbe();
  bool finishSample(char *line1, char *line2, size_t maxLen);
  void updateContact(uint16_t mv);
  bool updateResistive(uint16_t mv);
  void collectCalibration(uint16_t mv);
  void beginCalibrationWindow(Guided collect);
  void showMessage(const char *text);
  const char *guidedPrompt(bool firstLine) const;
  void formatLines(char *line1, char *line2, size_t maxLen, uint16_t mv) const;
  bool fail(const char *reason, bool measurementError);
  void releaseExcitation();
  void clearClassification();

  // Operation and excitation (only the two terminals of the probed pair).
  Operation operation_ = Operation::None;
  DriverOperationStatus status_ = DriverOperationStatus::Idle;
  const char *failureReason_ = "";
  bool measurementError_ = false;
  bool bound_ = false;  // pair known from a successful probe()
  uint8_t first_ = 0;
  uint8_t second_ = 0;
  uint8_t sensePin_ = 0;
  uint8_t sinkPin_ = 0;
  bool senseActive_ = false;  // ADC + pull-up configured on sensePin_
  bool sinkActive_ = false;   // open-drain LOW on sinkPin_
  bool reversed_ = false;     // current orientation
  Step step_ = Step::Idle;
  uint32_t stepSinceMs_ = 0;
  uint32_t waitMs_ = 0;
  uint8_t samples_ = 0;
  uint32_t sumMv_ = 0;
  uint16_t minMv_ = 0;
  uint16_t maxMv_ = 0;
  uint16_t resultMv_[2] = {0, 0};  // window average per orientation
  uint16_t spreadMv_[2] = {0, 0};

  // Classification of the probed pair (cleared by reset() and new probes).
  Kind kind_ = Kind::Unknown;
  bool closed_ = false;
  uint8_t debounce_ = 0;
  uint16_t baselineMv_ = 0;
  uint16_t filteredMv_ = 0;
  uint8_t variationRun_ = 0;
  uint16_t openRun_ = 0;
  bool openLike_ = false;

  // Guided LDR calibration.
  Guided guided_ = Guided::Off;
  uint32_t calSumMv_ = 0;
  uint16_t calMinMv_ = 0;
  uint16_t calMaxMv_ = 0;
  uint8_t calSamples_ = 0;
  uint16_t darkMv_ = 0;
  uint16_t brightMv_ = 0;
  const char *message_ = "";
  uint8_t messageLeft_ = 0;
};
