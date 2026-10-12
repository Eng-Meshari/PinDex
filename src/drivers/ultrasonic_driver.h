// HC-SR04-compatible ultrasonic driver: FourWirePowered with SIG1 = TRIG and
// SIG2 = ECHO. Trigger pulse, interrupt-timestamped echo capture,
// validation, confirmation and telemetry. Supply and routing are the
// engine's job, through the protected four-wire backend; this driver never
// powers anything. Two valid echoes show HC-SR04-compatible behaviour, not
// a specific model.
#pragma once

#include "../../sensor_driver.h"

// Validates one captured echo; every timestamp comes from micros(). Edges
// are measured from the trigger reference (the instant TRIG is released
// LOW) with unsigned arithmetic, so a micros() rollover is harmless. Returns
// nullptr if accepted (echoUs and distanceCm set), else the failure reason.
const char *ultrasonicCheckEcho(uint32_t triggerUs, uint32_t riseUs, uint32_t fallUs, uint32_t &echoUs,
                                float &distanceCm);

class UltrasonicDriver final : public SensorDriver {
 public:
  const char *getName() const override;
  bool probe(const ProbeContext &context) override;
  bool readSample(char *outLine1, char *outLine2, size_t maxLen) override;
  void reset() override;
  DriverOperationStatus getOperationStatus() const override;
  const DriverRequirements &getRequirements() const override;
  const char *getFailureReason() const override;

 private:
  enum class Operation : uint8_t { None, Probe, Sample };
  enum class Step : uint8_t { Idle, TriggerPreparation, WaitingForEcho };
  // Capture state shared with the ISR.
  enum Edge : uint8_t { kDisarmed, kAwaitRise, kAwaitFall, kComplete, kMalformed };

  static void onEchoEdge(void *self);
  void captureEdge();
  void begin(Operation operation);
  bool advance();
  void trigger();
  void setEdge(Edge edge);
  void wire();
  void release();
  bool fail(const char *reason);

  Operation operation_ = Operation::None;
  DriverOperationStatus status_ = DriverOperationStatus::Idle;
  Step step_ = Step::Idle;
  const char *failureReason_ = "";
  FourWirePins pins_ = {};
  bool bound_ = false;       // pins known from a successful probe()
  bool wired_ = false;       // TRIG driven LOW and the ECHO interrupt attached
  uint8_t accepted_ = 0;     // consecutive fresh valid echoes in this probe()
  bool triggered_ = false;   // triggerUs_ holds the previous trigger
  uint32_t triggerUs_ = 0;   // trigger reference of the current transaction
  uint32_t echoUs_ = 0;      // latest accepted echo
  float distanceCm_ = 0.0f;

  // Written by the ISR; armed, read and cleared under the capture lock.
  volatile uint8_t edge_ = kDisarmed;
  volatile uint32_t riseUs_ = 0;
  volatile uint32_t fallUs_ = 0;
};
