// Probe engine for the A/B/C test nodes. Owns the probe GPIOs and all
// detection state. With PINDEX_POWERED_PROBING_ENABLED == 0 (default) it
// only keeps the probes as high-impedance inputs.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "dht11.h"

#define PROBE_NODE_COUNT 3
#define PROBE_PERMUTATION_COUNT 6
#define PROBE_MAPPING_TEXT_LEN 22  // "A:VCC | B:DAT | C:GND" + NUL

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
  Scanning,     // precheck and permutation discovery
  Confirming,   // candidate mapping found, waiting for another valid frame
  Detected,     // DHT11-compatible confirmed, live readings
  Retrying,     // detected, but the latest read failed; reading is stale
  NoComponent,  // all six permutations failed: "No Component / Unknown"
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

struct ProbeStatus {
  ProbePhase phase;
  uint8_t scanIndex;             // 1..6 while scanning, else 0
  bool hasMapping;               // candidate or detected mapping saved
  ProbeMapping mapping;
  bool detected;                 // PROBE_CONFIRM_FRAMES consecutive valid frames
  bool readingValid;             // a reading exists for the saved mapping
  bool readingStale;             // the latest read failed; values are old
  int16_t temperatureDeciC;
  uint8_t humidityPct;
  uint32_t lastReadingMs;        // millis() of the last valid frame
  uint8_t consecutiveFailures;
  DhtError lastError;            // latest read error, None after a success
  PrecheckResult precheck;       // latest completed precheck
  uint32_t revision;             // changes whenever a displayed field changes
};

// Resets the probes to high impedance and starts the engine (Disabled or Idle).
void initProbeEngine();

// Sets every probe pin to plain INPUT (no pull-up/pull-down). Idempotent.
void resetTestPins();

// Advances the state machine; call on every loop() iteration. Never blocks
// on millisecond waits. The DHT capture step blocks for at most
// DHT_FRAME_DEADLINE_US (about 4-5 ms for a normal frame).
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
