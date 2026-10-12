// Passive two-terminal driver: resistive parts, contacts and switch-like
// behaviour, measured with weak internal pull-up excitation only (see
// weak_pull_meter.h). Classification is conservative and evidence-based; it
// never proves what the part is.
#pragma once

#include "../../sensor_driver.h"
#include "weak_pull_meter.h"  // also passiveEstimateOhms()

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
  WeakPullMeter meter_;       // the orientation being measured
  bool reversed_ = false;     // current orientation
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
