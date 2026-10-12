// I2C scanner driver: FourWirePowered with SIG1 = SDA and SIG2 = SCL on the
// Supply3V3 profile. Address-only sweeps of 0x08-0x77 (one transaction per
// call), two matching consecutive sweeps to confirm, then round-robin
// liveness checks of the confirmed addresses while monitoring. The bus
// lives behind i2c_bus.h; power is the engine's job. It reports which
// addresses answered and a hint per address: a common candidate, never a
// verified model.
#pragma once

#include "../../sensor_driver.h"

// 7-bit addresses 0x00-0x7F, as a bitmap plus count (never truncated).
struct I2cAddressSet {
  uint32_t bits[4];
  uint8_t count;
};

// Non-exclusive candidate description for an address ("MPU6050 / DS3231?"),
// or "Unknown model". An address alone never identifies a part.
const char *i2cAddressHint(uint8_t address);

class I2cDriver final : public SensorDriver {
 public:
  const char *getName() const override;
  bool probe(const ProbeContext &context) override;
  bool readSample(char *outLine1, char *outLine2, size_t maxLen) override;
  void reset() override;
  DriverOperationStatus getOperationStatus() const override;
  const DriverRequirements &getRequirements() const override;
  const char *getFailureReason() const override;
  bool failureIsMeasurementError() const override;
  bool resourcesReleased() const override;
  // "I2C ACK (n): 0x.. 0x..": every confirmed address, from cached results.
  bool formatDiagnostics(char *buf, size_t len) const override;

 private:
  enum class Operation : uint8_t { None, Probe, Sample };
  enum class Step : uint8_t { Idle, Sweep, Ping, Verify };

  bool begin(Operation operation);
  bool advance();
  bool advanceSweep();
  bool advancePing();
  bool linesIdle();
  void startSweep();
  bool succeed();
  bool fail(const char *reason, bool measurementError = false);
  void release();
  void clearResults();

  Operation operation_ = Operation::None;
  DriverOperationStatus status_ = DriverOperationStatus::Idle;
  Step step_ = Step::Idle;
  const char *failureReason_ = "";
  bool measurementError_ = false;
  FourWirePins pins_ = {};
  bool bound_ = false;          // confirmed addresses from a successful probe()
  bool busActive_ = false;      // the adapter's controller runs on pins_
  bool releaseFailed_ = false;  // a stop failed: ownership uncertain until one succeeds
  bool lineLow_ = false;        // SDA or SCL read LOW at lowSinceMs_, not released since
  uint32_t lowSinceMs_ = 0;

  // Discovery (probe()).
  uint8_t next_ = 0;              // next address of the sweep in progress
  uint8_t sweeps_ = 0;            // sweeps completed in this probe()
  uint32_t sweepStartMs_ = 0;
  I2cAddressSet sweep_ = {};      // the sweep in progress
  I2cAddressSet previous_ = {};   // the last completed sweep of this probe()

  // Confirmed result and monitoring (readSample()).
  I2cAddressSet confirmed_ = {};
  uint8_t shown_ = 0;    // address presented by the latest sample; 0 = none yet
  uint8_t pinging_ = 0;  // address of the liveness check in progress
};
