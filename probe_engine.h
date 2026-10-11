// Probe engine for the A/B/C test nodes. Owns the probe GPIOs, power
// sequencing and all detection policy; sensor protocols live in drivers
// (sensor_driver.h, src/drivers/). With PINDEX_POWERED_PROBING_ENABLED == 0
// it only keeps the probes as high-impedance inputs.
#pragma once

#include <stddef.h>
#include <stdint.h>

#define PROBE_NODE_COUNT 3
#define PROBE_PERMUTATION_COUNT 6
#define PROBE_MAPPING_TEXT_LEN 22    // "A:VCC | B:DAT | C:GND" + NUL
#define PROBE_TELEMETRY_TEXT_LEN 22  // one OLED line (21 glyphs) + NUL

// Socket roles for one permutation, as node indices (0 = A, 1 = B, 2 = C).
struct ProbeMapping {
  uint8_t vcc;
  uint8_t data;
  uint8_t gnd;
};

// Scan order, fixed: 1 ABC, 2 CBA, 3 BAC, 4 BCA, 5 ACB, 6 CAB (VCC DATA GND).
extern const ProbeMapping PROBE_PERMUTATIONS[PROBE_PERMUTATION_COUNT];

enum class ProbePhase : uint8_t {
  Disabled,     // powered probing compiled out; probes plain INPUT
  Idle,         // waiting before (re)scanning; nothing detected
  Scanning,     // precheck and permutation x driver discovery
  Confirming,   // candidate driver + mapping found, awaiting new acquisitions
  Detected,     // confirmed, live telemetry
  Retrying,     // detected, but the shown telemetry is stale
  NoComponent,  // every driver failed on every permutation
  Fault,        // precheck rejected the socket; latched until a retry
};

// Weak-pull precheck outcome. A diagnostic heuristic only: no outcome means
// the socket is safe to power.
enum class PrecheckResult : uint8_t {
  NotRun,
  NoObviousCoupling,     // nothing suspicious seen; NOT proof of no short
  SuspiciousCoupling,    // a pulled-down node read HIGH
  InconsistentReadings,  // a node changed between samples
};

// Snapshot of engine state and the latest result. Renderers use only this;
// nothing here can trigger an acquisition.
struct ProbeStatus {
  ProbePhase phase;
  uint8_t scanIndex;             // permutation 1..6 while scanning, else 0
  bool hasMapping;               // candidate or detected mapping saved
  ProbeMapping mapping;
  const char *componentName;     // driver getName() of the candidate/active driver, or nullptr
  bool detected;                 // PROBE_CONFIRM_FRAMES consecutive acquisitions
  bool readingValid;             // telemetry holds a completed sample
  bool readingStale;             // the latest acquisition failed; telemetry is old
  char telemetry1[PROBE_TELEMETRY_TEXT_LEN];  // driver-formatted, always terminated
  char telemetry2[PROBE_TELEMETRY_TEXT_LEN];  // "" if the driver uses one line
  uint32_t lastReadingMs;        // millis() of the last published sample
  uint8_t consecutiveFailures;
  const char *lastError;         // latest failure reason, nullptr after a success
  PrecheckResult precheck;       // latest completed precheck
  uint32_t revision;             // changes whenever a displayed field changes
};

// Resets the probes to high impedance and every registered driver, then
// starts the engine (Disabled or Idle).
void initProbeEngine();

// Sets every probe pin to plain INPUT (no pull-up/pull-down). Idempotent.
void resetTestPins();

// Advances the state machine; call on every loop() iteration. Never blocks
// on millisecond waits; only a driver's documented, bounded capture may
// block briefly inside a probe()/readSample() call.
void probeTick();

const ProbeStatus &probeStatus();

// Clears a latched precheck fault and reruns the full precheck and scan.
// Returns false (and does nothing) if there is no fault or powered probing
// is disabled; it never bypasses PINDEX_POWERED_PROBING_ENABLED.
bool probeRequestRetry();

// Writes "A:VCC | B:DAT | C:GND" style text, always in A/B/C order.
void formatProbeMapping(const ProbeMapping &mapping, char *buf, size_t len);

const char *probePhaseName(ProbePhase phase);
const char *precheckResultName(PrecheckResult result);
