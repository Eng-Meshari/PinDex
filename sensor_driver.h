// Abstract sensor driver contract for the three-node (VCC/DATA/GND) socket.
//
// Ownership: the probe engine owns power (VCC/GND sequencing), prechecks,
// permutation choice, teardown, scheduling, confirmation and removal policy.
// A driver owns only protocol signalling on the DATA pin, capture, decoding,
// validation, its own sample storage and telemetry formatting. A driver must
// never power VCC/GND, choose another permutation, touch the OLED or modify
// engine state.
//
// Cooperative operations (probe() and readSample()):
//   * The first call after reset() or after a completed operation starts a
//     new operation; later calls advance that same pending operation.
//   * Arguments must not change while an operation is pending.
//   * true  = an operation that has just completed successfully.
//     false = inspect getOperationStatus(): Pending is NOT a failure.
//   * Once an operation completes (Succeeded/Failed) the engine stops
//     polling it; the next call starts a new, genuinely new acquisition.
//   * The engine never overlaps probe and sample operations. A driver treats
//     an overlapping or argument-changing call as a failed operation.
//   * readSample() needs a prior successful probe(); reset() forgets it.
//   * Long waits return Pending. Only a short, bounded timing-critical
//     capture may block (the DHT11 capture blocks at most 6.5 ms).
//
// readSample() buffers: maxLen is the capacity of EACH buffer including the
// terminator. Null pointers or zero capacity are rejected without being
// dereferenced. For valid buffers, both are always NUL-terminated on return
// and hold text only after a newly completed successful sample; on Pending
// or failure they are empty. Drivers never keep the caller's pointers.
//
// reset(): idempotent; cancels any pending operation and clears protocol,
// binding and sample state. It may only make a pin safer (e.g. release a
// line it is holding low to plain INPUT); it never energises a pin and
// never touches another driver's resources.
#pragma once

#include <stddef.h>
#include <stdint.h>

enum class DriverOperationStatus : uint8_t {
  Idle,       // nothing started since reset()
  Pending,    // in progress; call again later
  Succeeded,  // the latest operation completed successfully
  Failed      // the latest operation failed; see getFailureReason()
};

// Fixed per driver; the engine schedules power and acquisitions from it.
struct DriverRequirements {
  uint32_t powerStabilizeMs;          // power-up to the first probe() call
  uint32_t minAcquisitionIntervalMs;  // between operation starts, incl. retries
  uint32_t sampleIntervalMs;          // readSample() cadence (never below the minimum)
  uint32_t operationTimeoutMs;        // the engine fails an operation pending this long
};

class SensorDriver {
 public:
  virtual ~SensorDriver() = default;

  // Stable string that lives as long as the driver (the firmware).
  virtual const char *getName() const = 0;

  // GPIO numbers of the powered permutation. Succeeds only after a genuine
  // acquisition proves the part speaks this driver's protocol.
  virtual bool probe(uint8_t pinVcc, uint8_t pinData, uint8_t pinGnd) = 0;

  // New acquisition on the probed pins, formatted into two lines (the
  // second may be empty). See the buffer rules above.
  virtual bool readSample(char *outLine1, char *outLine2, size_t maxLen) = 0;

  virtual void reset() = 0;

  virtual DriverOperationStatus getOperationStatus() const = 0;

  virtual const DriverRequirements &getRequirements() const = 0;

  // Short stable reason for the latest failure, for logs; "" if none.
  virtual const char *getFailureReason() const { return ""; }
};
