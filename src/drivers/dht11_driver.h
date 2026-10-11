// DHT11-compatible driver: start pulse, bounded single-wire capture and
// decoding (via dht11.h) on the DATA pin only. Power is the engine's job.
#pragma once

#include "../../dht11.h"
#include "../../sensor_driver.h"

class Dht11Driver final : public SensorDriver {
 public:
  const char *getName() const override;
  bool probe(uint8_t pinVcc, uint8_t pinData, uint8_t pinGnd) override;
  bool readSample(char *outLine1, char *outLine2, size_t maxLen) override;
  void reset() override;
  DriverOperationStatus getOperationStatus() const override;
  const DriverRequirements &getRequirements() const override;
  const char *getFailureReason() const override;

 private:
  enum class Operation : uint8_t { None, Probe, Sample };

  void begin(Operation operation);
  bool advance();
  bool fail(const char *reason);
  void releaseHeldLine();

  Operation operation_ = Operation::None;
  DriverOperationStatus status_ = DriverOperationStatus::Idle;
  bool bound_ = false;      // pins known from a successful probe()
  bool holdingLow_ = false;  // this driver is sinking DATA (start pulse)
  uint8_t pinVcc_ = 0;
  uint8_t pinData_ = 0;
  uint8_t pinGnd_ = 0;
  uint32_t startMs_ = 0;
  DhtPulses pulses_ = {};
  DhtReading reading_ = {};
  const char *failureReason_ = "";
};
