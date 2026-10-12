// Probe engine for the A/B/C/D test nodes. Owns the probe GPIOs, the
// precheck routing policy, power sequencing (including the four-wire
// backend) and all detection policy; sensor protocols live in drivers
// (sensor_driver.h, src/drivers/). With PINDEX_POWERED_PROBING_ENABLED and
// PINDEX_PASSIVE_PROBING_ENABLED both 0 it only keeps the probes as
// high-impedance inputs.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "config.h"
#include "sensor_driver.h"

#define PROBE_PERMUTATION_COUNT 24   // three-wire: ordered VCC/DATA/GND on 3 of 4 nodes
#define PROBE_FOUR_WIRE_COUNT 24     // four-wire: ordered VCC/SIG1/SIG2/GND on all 4 nodes
#define PROBE_PAIR_COUNT 6           // two-terminal: unordered node pairs
#define PROBE_MAPPING_TEXT_LEN 16    // one half of the mapping text, "A:VCC B:TRG" + NUL
#define PROBE_TELEMETRY_TEXT_LEN 22  // one OLED line (21 glyphs) + NUL

// What each physical socket node does in one attempt.
enum class NodeRole : uint8_t { Unused, Vcc, Data, Gnd, Terminal, Sig1, Sig2 };

// Roles of nodes A, B, C, D (index 0..3). Socket labels stay separate from
// GPIO numbers; the engine resolves GPIOs per node.
struct ProbeMapping {
  ProbeTopology topology;
  NodeRole role[PROBE_NODE_COUNT];
};

// The canonical node table: node i is labelled PROBE_NODE_LABEL[i] on the
// socket and wired to GPIO PROBE_NODE_GPIO[i].
extern const uint8_t PROBE_NODE_GPIO[PROBE_NODE_COUNT];
extern const char PROBE_NODE_LABEL[PROBE_NODE_COUNT];

// Three-wire scan order, fixed: the original six A/B/C permutations first
// (1 ABC, 2 CBA, 3 BAC, 4 BCA, 5 ACB, 6 CAB as VCC DATA GND), then the same
// six patterns on the node sets ABD (7-12), ACD (13-18) and BCD (19-24).
// The fourth node is unused.
extern const ProbeMapping PROBE_PERMUTATIONS[PROBE_PERMUTATION_COUNT];

// Four-wire scan order: every assignment of A..D to (VCC, SIG1, SIG2, GND),
// lexicographic by node: ABCD, ABDC, ACBD, ... DCBA.
extern const ProbeMapping PROBE_FOUR_WIRE_MAPPINGS[PROBE_FOUR_WIRE_COUNT];

// Two-terminal pairs: A-B, B-C, A-C (the original order), then A-D, B-D,
// C-D; the other two nodes are unused.
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
                         // drive across it (except the measured open-drain
                         // pull-up exception, decideCoupling()); passive
                         // measurement may proceed
  InconsistentReadings,  // still noisy after PRECHECK_MAX_RETRIES: unknown
};

// Derived evidence for one physical pair, from a weak-pull ADC measurement
// in both orientations with every supply off. Never derived from the
// digital CouplingObserved flag alone.
enum class CouplingClassification : uint8_t {
  Unclassified,      // not measured in this observation cycle
  PullupCompatible,  // stable, symmetric, inside the supported single pull-up band
  Resistive,         // stable, symmetric, finite, outside that band (e.g. two pull-ups in series)
  LowImpedance,      // a contact-band reading in either orientation: cannot be told from a short
  OpenLike,          // stable, symmetric, at/above the open band: no finite path seen
  Ambiguous,         // unstable or directional (junctions, regulators, charging capacitance)
  MeasurementError,  // ADC unavailable/failed, or the pull-up could not be verified
};

struct PairEvidence {
  CouplingClassification classification;
  uint16_t mv[2];  // window mean, sensing the pair's first terminal, then its second
                   // (real measurements only; 0 when not measured or on error)
};

enum class CouplingVerdict : uint8_t {
  Allowed,           // no coupling on a path the attempt drives (the existing rule)
  PullupException,   // allowed by the narrow open-drain pull-up exception
  NeedsMeasurement,  // the exception may apply once `measure` is classified
  Blocked,           // vetoed; `reason` says why
};

struct CouplingDecision {
  CouplingVerdict verdict;
  uint8_t measure;  // NeedsMeasurement: PROBE_PAIRS bits still to classify
  char reason[48];  // e.g. "VCC-GND coupling", "open-drain pull-up exception"
};

// Classifies one pair from its two orientation means and window spreads.
CouplingClassification classifyCouplingPair(const uint16_t mv[2], const uint16_t spreadMv[2]);

// True if the SIG1-SIG2 reading matches the VCC-SIG1 and VCC-SIG2 pull-ups
// in series (all mean mV), within PULLUP_SERIES_TOLERANCE_PCT.
bool pullupSeriesExplained(uint16_t vccSig1Mv, uint16_t vccSig2Mv, uint16_t sig1Sig2Mv);

// The precheck coupling policy for a powered (ProtectedActive) candidate
// whose capabilities and supply are already permitted. Order: VCC-GND
// coupling is always vetoed; no coupling on a driven path is allowed (the
// existing rule); otherwise only a FourWirePowered + Supply3V3 +
// open-drain candidate whose VCC is coupled to SIG1/SIG2 alone may use the
// pull-up exception, and only after VCC-GND, VCC-SIG1, VCC-SIG2 and
// SIG1-SIG2 are classified in this observation cycle (`measured`,
// `pairEvidence`): the rail must measure finite-high or open (pull-ups can
// mask a rail short from the digital test), both VCC-signal pairs
// PullupCompatible and SIG1-SIG2 their series path. Physical pairs come
// from `m`'s roles every time; nothing is cached per role. Passing means
// only that this policy did not veto the attempt.
CouplingDecision decideCoupling(const DriverRequirements &r, const ProbeMapping &m, uint8_t coupled,
                                uint8_t measured, const PairEvidence pairEvidence[]);

// The latest coupling-policy evaluation of a powered candidate that was
// blocked by coupling or allowed by the exception (diagnostics only).
struct CouplingEvaluation {
  uint32_t serial;               // changes with every new evaluation; 0 = none yet
  ProbeMapping mapping;          // candidate mapping it was evaluated for
  const char *driverName;
  const char *signalLabel[2];
  uint8_t coupledPairs;          // raw digital observation of that precheck pass
  uint8_t measuredPairs;         // pairs classified in that pass
  PairEvidence pair[PROBE_PAIR_COUNT];
  CouplingVerdict verdict;
  char reason[48];
};

// Snapshot of engine state and the latest result. Renderers use only this;
// nothing here can trigger an acquisition.
struct ProbeStatus {
  ProbePhase phase;
  uint16_t scanIndex;            // attempt 1..scanTotal while scanning, else 0
  uint16_t scanTotal;            // attempts per discovery cycle
  bool hasMapping;               // candidate or detected mapping saved
  ProbeMapping mapping;
  const char *signalLabel[2];    // SIG1/SIG2 names of a four-wire mapping (driver's), else nullptr
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
  uint8_t hardwareRequiredMask;  // bit i: registered driver i is enabled but has no eligible
                                 // supply (no capable backend, profile off); it is skipped
  CouplingEvaluation evaluation; // latest coupling evaluation (Serial log; not displayed)
  uint32_t revision;             // changes whenever a displayed field changes
};

// Resets the probes to high impedance and every registered driver, then
// starts the engine (Disabled or Idle).
void initProbeEngine();

// Sets every probe pin to plain INPUT (no pull-up/pull-down), then switches
// the four-wire backend off (supply off, routes open). Idempotent.
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

// The detected driver's diagnostic text (cached results only; no bus or pin
// activity), e.g. every I2C address found. buf is always terminated; false
// if nothing is detected or the driver has nothing to report.
bool probeFormatDiagnostics(char *buf, size_t len);

// Mapping text in socket order, as two halves that each fit one OLED line
// (the joined four-node text does not, at the size-1 font):
//   four-wire  "A:VCC B:TRG" + "C:ECH D:GND"  (SIG names from signalLabel,
//                                              "S1"/"S2" if null)
//   three-wire "A:VCC B:DAT" + "C:GND D:---"
//   pair       "Pins: A & B" + "C:--- D:---"
// len is the capacity of each buffer; both are always terminated.
void formatProbeMapping(const ProbeMapping &mapping, const char *const *signalLabel, char *first,
                        char *second, size_t len);

// "VCC", "DAT", "GND", "T", the signal label (or "S1"/"S2"), "---".
const char *probeRoleName(NodeRole role, const char *const *signalLabel);

const char *probePhaseName(ProbePhase phase);
const char *precheckResultName(PrecheckResult result);
const char *couplingClassificationName(CouplingClassification c);
