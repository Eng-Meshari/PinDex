// Abstract sensor driver contract for the three-node (A/B/C) socket.
//
// Ownership: the probe engine owns pin permission, prechecks, which nodes
// an attempt may use, VCC/GND power sequencing (powered topology only),
// teardown, scheduling, confirmation and removal policy. A driver owns only
// the signalling it needs on the nodes handed to it in a ProbeContext:
// protocol timing, capture, decoding, validation, its own sample storage
// and telemetry formatting. A driver never powers VCC/GND, picks other
// nodes, touches the OLED or modifies engine state.
//
// Capabilities (DriverRequirements::topology / ::excitation) decide how
// the engine prepares an attempt; drivers are never selected by name:
//   ThreeWirePowered + ProtectedActive: engine drives GND LOW, VCC HIGH and
//     pulls DATA up (behind the protected interface) before probe().
//   TwoTerminal + WeakPullOnly: engine powers nothing; every node is plain
//     INPUT and the unused node stays that way. The driver may only use an
//     internal pull-up and an open-drain LOW sink on its two terminals.
//   Any other combination is unsupported and never attempted.
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
//     an overlapping, argument-changing or wrong-topology call as a failed
//     operation without energising anything.
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
// binding, calibration and sample state. It may only make a pin safer
// (e.g. release a line it is holding low or a pull-up it enabled, to plain
// INPUT); it never energises a pin and never touches another driver's pins.
#pragma once

#include <stddef.h>
#include <stdint.h>

enum class DriverOperationStatus : uint8_t {
  Idle,       // nothing started since reset()
  Pending,    // in progress; call again later
  Succeeded,  // the latest operation completed successfully
  Failed      // the latest operation failed; see getFailureReason()
};

enum class ProbeTopology : uint8_t {
  ThreeWirePowered,  // VCC / DATA / GND roles over the six permutations
  TwoTerminal,       // two terminals plus an unused node, over the three pairs
};

enum class ExcitationPolicy : uint8_t {
  ProtectedActive,  // engine-driven supply; needs the protected interface
  WeakPullOnly,     // internal pull-up + open-drain LOW sink; never drives HIGH
};

// Fixed per driver; the engine prepares and schedules attempts from it.
struct DriverRequirements {
  ProbeTopology topology;
  ExcitationPolicy excitation;
  uint32_t powerStabilizeMs;          // preparation to the first probe() call
  uint32_t minAcquisitionIntervalMs;  // between operation starts, incl. retries
  uint32_t sampleIntervalMs;          // readSample() cadence (never below the minimum)
  uint32_t operationTimeoutMs;        // the engine fails an operation pending this long
};

// GPIO numbers for one attempt. Only the half matching `topology` is valid;
// a passive terminal is never passed as "VCC".
struct ThreeWirePins {
  uint8_t vcc;
  uint8_t data;
  uint8_t gnd;
};

struct TwoTerminalPins {
  uint8_t first;   // terminals of the selected pair, in socket order
  uint8_t second;
  uint8_t unused;  // plain INPUT; the driver must not touch it
};

struct ProbeContext {
  ProbeTopology topology;
  ThreeWirePins powered;  // ThreeWirePowered only
  TwoTerminalPins pair;   // TwoTerminal only
};

class SensorDriver {
 public:
  virtual ~SensorDriver() = default;

  // Static string valid for the firmware's lifetime. It may change as the
  // driver's classification of the part evolves; the engine re-reads it.
  virtual const char *getName() const = 0;

  // Succeeds only after a genuine acquisition shows the part behaves as this
  // driver expects on these pins.
  virtual bool probe(const ProbeContext &context) = 0;

  // New acquisition on the probed pins, formatted into two lines (the
  // second may be empty). See the buffer rules above.
  virtual bool readSample(char *outLine1, char *outLine2, size_t maxLen) = 0;

  virtual void reset() = 0;

  virtual DriverOperationStatus getOperationStatus() const = 0;

  virtual const DriverRequirements &getRequirements() const = 0;

  // Short stable reason for the latest failure, for logs; "" if none.
  virtual const char *getFailureReason() const { return ""; }

  // True if the latest failure means the measurement itself was unavailable
  // (e.g. ADC error) rather than "this part does not match".
  virtual bool failureIsMeasurementError() const { return false; }

  // Optional user command for the detected driver (e.g. a guided
  // calibration step). Returns true if the driver accepted it.
  virtual bool handleUserCommand(char command) {
    (void)command;
    return false;
  }
};
