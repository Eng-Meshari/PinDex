// Abstract sensor driver contract for the four-node (A/B/C/D) socket.
//
// Ownership: the probe engine owns pin permission, prechecks, which nodes
// an attempt may use, power sequencing (GPIO VCC/GND for three-wire, the
// protected backend or the experimental GPIO profile for four-wire),
// teardown, scheduling, confirmation and
// removal policy. A driver owns only the signalling it needs on the nodes
// handed to it in a ProbeContext: protocol timing, capture, decoding,
// validation, its own sample storage and telemetry formatting. A driver
// never powers VCC/GND, picks other nodes, touches the OLED or modifies
// engine state.
//
// Capabilities (DriverRequirements::topology / ::excitation / ::supply)
// decide how the engine prepares an attempt; drivers are never selected by
// name:
//   ThreeWirePowered + ProtectedActive + GpioLogic: engine drives GND LOW,
//     VCC HIGH and pulls DATA up (behind the protected interface) before
//     probe(); the fourth node stays plain INPUT. A driver whose DATA is the
//     module's output (dataIsOutput) gets no engine pull-up; the node is
//     shown as OUT and the driver chooses its input bias.
//   FourWirePowered + ProtectedActive + Backend5V / Supply3V3: the
//     protected four-wire backend (four_wire_backend.h) routes that supply
//     and ground, only if it reports kBackendFourWireBaseline plus the
//     supply's bit; every GPIO stays plain INPUT. Supply3V3 alone may
//     instead use the experimental GPIO profile (GND LOW, VCC HIGH; off by
//     default). The driver signals on SIG1/SIG2 only.
//   TwoTerminal + WeakPullOnly + None: engine powers nothing; every node is
//     plain INPUT and the two unused nodes stay that way. The driver may
//     only use an internal pull-up and an open-drain LOW sink on its two
//     terminals.
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
//     capture may block (the DHT11 capture blocks at most 6.5 ms; the
//     ultrasonic trigger pulse ~10 us, its echo is captured by interrupt;
//     one synchronous I2C transaction per call, bounded by its timeout).
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
// INPUT, detach an interrupt it attached, or stop a bus peripheral it
// started); it never energises a pin and never touches another driver's
// pins. On teardown the engine calls it FIRST, while the attempt is still
// powered, so signals and peripherals are released before the supply is
// removed; then it removes power and restores every node to plain INPUT.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "config.h"

enum class DriverOperationStatus : uint8_t {
  Idle,       // nothing started since reset()
  Pending,    // in progress; call again later
  Succeeded,  // the latest operation completed successfully
  Failed      // the latest operation failed; see getFailureReason()
};

enum class ProbeTopology : uint8_t {
  ThreeWirePowered,  // VCC / DATA / GND on three of the four nodes (24 mappings)
  TwoTerminal,       // two terminals, two unused nodes (6 pairs)
  FourWirePowered,   // VCC / SIG1 / SIG2 / GND on all four nodes (24 mappings)
};

enum class ExcitationPolicy : uint8_t {
  ProtectedActive,  // engine-switched supply; needs the protected interface
  WeakPullOnly,     // internal pull-up + open-drain LOW sink; never drives HIGH
};

// What the protected four-wire backend actually provides, as reported by
// fourWireBackendCapabilities(). Only the backend reports these; a config
// flag never grants one.
constexpr uint16_t kBackendSupply5V = 1u << 0;              // switched 5 V, current-limited, enough for the sensor
constexpr uint16_t kBackendSupplySwitching = 1u << 1;       // supply routed to any chosen node, and off on command
constexpr uint16_t kBackendGroundRouting = 1u << 2;         // return path routed to the chosen node only
constexpr uint16_t kBackendSignalLevels = 1u << 3;          // no node can put more than 3.3 V on its GPIO
constexpr uint16_t kBackendAssignmentProtection = 1u << 4;  // every wrong candidate mapping tolerated
constexpr uint16_t kBackendBreakBeforeMake = 1u << 5;       // all routes open before a new mapping closes
constexpr uint16_t kBackendSupply3V3 = 1u << 6;             // switched 3.3 V, current-limited, enough for the module

// Every FourWirePowered attempt needs these, whatever the sensor.
constexpr uint16_t kBackendFourWireBaseline = kBackendSupplySwitching | kBackendGroundRouting |
                                              kBackendSignalLevels | kBackendAssignmentProtection |
                                              kBackendBreakBeforeMake;

// Where a powered attempt's supply comes from. The engine prepares power
// from this; a driver never does. Three properties stay separate: 3.3 V
// logic compatibility, whether the whole module accepts the supply's
// voltage and current, and tolerance of wrong assignments while scanning.
enum class SupplyProfile : uint8_t {
  None,       // TwoTerminal: nothing is ever powered
  GpioLogic,  // ThreeWirePowered: VCC HIGH / GND LOW from probe GPIOs (nominal 3.3 V logic level)
  Backend5V,  // FourWirePowered: the protected backend's switched 5 V only; never GPIOs
  Supply3V3,  // FourWirePowered: the protected backend's switched 3.3 V; without one, only the
              // experimental bare-GPIO profile (PINDEX_I2C_GPIO_POWER_ENABLED, off by default)
};

// Fixed per driver; the engine prepares and schedules attempts from it.
struct DriverRequirements {
  ProbeTopology topology;
  ExcitationPolicy excitation;
  uint32_t powerStabilizeMs;          // preparation to the first probe() call
  uint32_t minAcquisitionIntervalMs;  // between operation starts, incl. retries
  uint32_t sampleIntervalMs;          // readSample() cadence (never below the minimum)
  uint32_t operationTimeoutMs;        // the engine fails an operation pending this long
  SupplyProfile supply;               // must match the topology (see the engine's supported())
  bool openDrainSignals;              // signal/DATA nodes are only released or sunk, never driven
                                      // HIGH (a push-pull signal is vetoed on any coupling)
  const char *signalLabel[2];         // FourWirePowered: display names of SIG1, SIG2 (<= 3 chars); else nullptr
  bool dataIsOutput;                  // ThreeWirePowered: DATA is the module's output, only ever read
                                      // (never sunk, no engine pull-up); shown as OUT
};

// How much a successful probe() says about the part. The engine stops
// discovery only on Validated; a Candidate is kept tentative while every
// other eligible attempt still runs, and is never shown as confirmed.
enum class EvidenceLevel : uint8_t {
  None,          // nothing since reset()
  Candidate,     // plausible, not identifying (e.g. a driven, stable digital level)
  Validated,     // a protocol response the driver checked (DHT frame, echoes, I2C ACKs)
  UserVerified,  // a candidate the user confirmed with a guided test on this binding
};

// GPIO numbers for one attempt. Only the member matching `topology` is
// valid; a passive terminal is never passed as "VCC".
struct ThreeWirePins {
  uint8_t vcc;
  uint8_t data;
  uint8_t gnd;
};

struct TwoTerminalPins {
  uint8_t first;   // terminals of the selected pair, in socket order
  uint8_t second;
  uint8_t unused[PROBE_NODE_COUNT - 2];  // plain INPUT; the driver must not touch them
};

struct FourWirePins {
  uint8_t vcc;   // supplied by the engine (backend, or GPIO HIGH in the experimental profile)
  uint8_t sig1;  // e.g. ultrasonic TRIG, I2C SDA
  uint8_t sig2;  // e.g. ultrasonic ECHO, I2C SCL
  uint8_t gnd;   // routed by the engine (backend, or GPIO LOW in the experimental profile)
};

struct ProbeContext {
  ProbeTopology topology;
  ThreeWirePins powered;  // ThreeWirePowered only
  TwoTerminalPins pair;   // TwoTerminal only
  FourWirePins fourWire;  // FourWirePowered only
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

  // False if reset() could not release a peripheral it acquired (e.g. an
  // I2C controller that refused to stop): ownership is uncertain, so the
  // engine powers down and latches a fault instead of starting another
  // attempt. Each later reset() tries the release once more.
  virtual bool resourcesReleased() const { return true; }

  // Optional diagnostic text built from cached results only (never a bus
  // or pin operation), e.g. every address found. Terminated, bounded by
  // len; false if there is nothing to report.
  virtual bool formatDiagnostics(char *buf, size_t len) const {
    (void)buf;
    (void)len;
    return false;
  }

  // Optional user command for the detected driver (e.g. a guided
  // calibration step). Returns true if the driver accepted it.
  virtual bool handleUserCommand(char command) {
    (void)command;
    return false;
  }

  // Evidence behind the latest successful probe() or readSample(); the
  // engine reads it only after one. Protocol drivers are Validated.
  virtual EvidenceLevel evidence() const { return EvidenceLevel::Validated; }
};
