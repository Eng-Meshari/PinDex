// Probe engine for the A/B/C test nodes. Owns the probe GPIOs, the
// precheck routing policy, power sequencing and all detection policy;
// sensor protocols live in drivers (sensor_driver.h, src/drivers/).
// With PINDEX_POWERED_PROBING_ENABLED and PINDEX_PASSIVE_PROBING_ENABLED
// both 0 it only keeps the probes as high-impedance inputs.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "sensor_driver.h"

#define PROBE_NODE_COUNT 3
#define PROBE_PERMUTATION_COUNT 6
#define PROBE_PAIR_COUNT 3
#define PROBE_MAPPING_TEXT_LEN 22    // "A:VCC | B:DAT | C:GND" + NUL
#define PROBE_TELEMETRY_TEXT_LEN 22  // one OLED line (21 glyphs) + NUL

// What each physical socket node does in one attempt.
enum class NodeRole : uint8_t { Unused, Vcc, Data, Gnd, Terminal };

// Roles of nodes A, B, C (index 0, 1, 2). Socket labels stay separate from
// GPIO numbers; the engine resolves GPIOs per node.
struct ProbeMapping {
  ProbeTopology topology;
  NodeRole role[PROBE_NODE_COUNT];
};

// Powered scan order, fixed: 1 ABC, 2 CBA, 3 BAC, 4 BCA, 5 ACB, 6 CAB (VCC DATA GND).
extern const ProbeMapping PROBE_PERMUTATIONS[PROBE_PERMUTATION_COUNT];

// Two-terminal pairs, in order A-B, B-C, A-C; the third node is unused.
extern const ProbeMapping PROBE_PAIRS[PROBE_PAIR_COUNT];

// Bit i of ProbeStatus::coupledPairs refers to PROBE_PAIRS[i].
uint8_t probePairBit(uint8_t nodeX, uint8_t nodeY);

// Index of the nth node with `role` in `mapping`, or -1.
int probeNodeWithRole(const ProbeMapping &mapping, NodeRole role, int nth = 0);

enum class ProbePhase : uint8_t {
  Disabled,     // all probing compiled out; probes plain INPUT
  Idle,         // waiting before (re)scanning; nothing detected
  Scanning,     // precheck and attempt discovery
  Confirming,   // candidate driver + mapping found, awaiting new acquisitions
  Detected,     // confirmed, live telemetry
  Retrying,     // detected, but the shown telemetry is stale
  NoComponent,  // nothing matched (or the observations were ambiguous)
  Fault,        // latched internal fault; probes INPUT until a retry
};

// Weak-pull precheck outcome. A routing observation, never a safety pass.
enum class PrecheckResult : uint8_t {
  NotRun,
  NoObviousCoupling,     // no pulled-down node read HIGH; NOT proof of no short
  CouplingObserved,      // some pair coupled (contact or finite R): no powered
                         // drive across it; passive measurement may proceed
  InconsistentReadings,  // still noisy after PRECHECK_MAX_RETRIES: unknown
};

// Snapshot of engine state and the latest result. Renderers use only this;
// nothing here can trigger an acquisition.
struct ProbeStatus {
  ProbePhase phase;
  uint8_t scanIndex;             // attempt 1..scanTotal while scanning, else 0
  uint8_t scanTotal;             // attempts per discovery cycle
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
  uint8_t coupledPairs;          // pairs the latest precheck saw coupled
  bool measurementUnavailable;   // last cycle failed on measurement errors (e.g. ADC)
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

// Clears a latched fault and reruns the full precheck and scan. Returns
// false (and does nothing) if there is no fault or probing is disabled.
bool probeRequestRetry();

// Forgets the current result (e.g. a switch left OPEN, which cannot be told
// apart from removal), makes the pins safe and rescans now. False if
// probing is disabled.
bool probeRequestClear();

// Forwards a user command to the detected driver. False if no driver is
// detected or it did not accept the command.
bool probeSendCommand(char command);

// "A:VCC | B:DAT | C:GND" or "Pins: A & B, C:---", always in A/B/C order.
void formatProbeMapping(const ProbeMapping &mapping, char *buf, size_t len);

const char *probePhaseName(ProbePhase phase);
const char *precheckResultName(PrecheckResult result);
