// PinDex - Smart Component & Sensor Identifier
// Sensor drivers (src/drivers/) behind a generic probe engine on a
// four-node A/B/C/D socket: a DHT11-compatible three-wire driver, an I2C
// address scanner and an HC-SR04-compatible driver (four-wire; both need a
// protected backend this board does not have, the I2C scanner optionally
// the experimental PINDEX_I2C_GPIO_POWER_ENABLED profile) and a weak-pull
// passive/switch driver. Powered and passive probing are controlled
// separately in config.h (PINDEX_POWERED_PROBING_ENABLED,
// PINDEX_PASSIVE_PROBING_ENABLED); read the README before enabling any.

#include "config.h"
#include "display_manager.h"
#include "probe_engine.h"
#include "src/drivers/driver_registry.h"

static bool displayOk = false;
static bool splashPending = false;  // true until the probe screens take over
static unsigned long splashStartMs = 0;
static unsigned long lastHeartbeatMs = 0;
static uint32_t shownRevision = 0;
static uint32_t loggedRevision = 0;
static bool loggedDetected = false;
static uint32_t loggedEvaluation = 0;
static uint16_t loggedInconclusive = 0;
static unsigned long lastBlockedLogMs = 0;
static bool blockedLogged = false;

// The detected driver's cached diagnostics (e.g. every I2C address found);
// prints nothing if there are none. Never triggers bus or pin activity.
static void printDiagnostics() {
  static char text[640];
  if (probeFormatDiagnostics(text, sizeof text)) Serial.printf("diagnostics: %s\n", text);
}

static void logProbeStatus(const ProbeStatus &s) {
  Serial.printf("probe: %s", probePhaseName(s.phase));
  if (s.phase == ProbePhase::Scanning) {
    Serial.printf(" %u/%u", (unsigned)s.scanIndex, (unsigned)s.scanTotal);
  }
  if (s.precheck == PrecheckResult::CouplingObserved) {
    Serial.print(" coupled=");
    for (uint8_t i = 0; i < PROBE_PAIR_COUNT; i++) {
      if ((s.coupledPairs & (1u << i)) == 0) continue;
      const ProbeMapping &p = PROBE_PAIRS[i];
      Serial.printf("%c%c ", PROBE_NODE_LABEL[probeNodeWithRole(p, NodeRole::Terminal, 0)],
                    PROBE_NODE_LABEL[probeNodeWithRole(p, NodeRole::Terminal, 1)]);
    }
  }
  if (s.componentName != nullptr) Serial.printf(" %s", s.componentName);
  if (s.hasMapping) {
    char first[PROBE_MAPPING_TEXT_LEN];
    char second[PROBE_MAPPING_TEXT_LEN];
    formatProbeMapping(s.mapping, s.signalLabel, first, second, sizeof first);
    Serial.printf(" [%s %s]", first, second);
  }
  if (s.readingValid) {
    Serial.printf(" \"%s\"", s.telemetry1);
    if (s.telemetry2[0] != '\0') Serial.printf(" \"%s\"", s.telemetry2);
    if (s.readingStale) Serial.print(" STALE");
  }
  if (s.consecutiveFailures > 0) Serial.printf(" fails=%u", (unsigned)s.consecutiveFailures);
  if (s.lastError != nullptr) Serial.printf(" err=%s", s.lastError);
  // Skipped attempts were never tested: not evidence of an empty socket.
  if (s.inconclusiveCount > 0 && (s.phase == ProbePhase::Scanning || s.phase == ProbePhase::NoComponent)) {
    Serial.printf(" inconclusive=%u/%u", (unsigned)s.inconclusiveCount, (unsigned)s.scanTotal);
  }
  if (s.measurementUnavailable) Serial.print(" measurement-unavailable");
  for (size_t i = 0; i < sensorDriverCount() && i < 8; i++) {
    if (s.hardwareRequiredMask & (1u << i)) Serial.printf(" hw-required=[%s]", sensorDriverAt(i)->getName());
  }
  if (s.phase == ProbePhase::Fault) {
    Serial.printf(" precheck=%s; send 'r' to retry", precheckResultName(s.precheck));
  }
  Serial.println();
  if (s.detected && !loggedDetected) printDiagnostics();  // once per detection
  loggedDetected = s.detected;
}

// Coupling-policy evidence for a powered candidate the precheck saw coupled:
// its mapping, each relevant pair (raw digital observation, then the
// weak-pull ADC classification and its two orientation readings when one
// was measured) and the outcome. A blocked evaluation is printed at most
// once per PRECHECK_LOG_INTERVAL_MS; one allowed by the exception always.
static void logEvaluation(const CouplingEvaluation &e, unsigned long now) {
  const bool allowed = e.verdict == CouplingVerdict::PullupException;
  if (!allowed && blockedLogged && now - lastBlockedLogMs < PRECHECK_LOG_INTERVAL_MS) return;
  if (!allowed) {
    lastBlockedLogMs = now;
    blockedLogged = true;
  }
  char first[PROBE_MAPPING_TEXT_LEN];
  char second[PROBE_MAPPING_TEXT_LEN];
  formatProbeMapping(e.mapping, e.signalLabel, first, second, sizeof first);
  Serial.printf("precheck: %s %s (%s)\n", first, second, e.driverName);
  for (uint8_t i = 0; i < PROBE_PAIR_COUNT; i++) {
    const uint8_t bit = static_cast<uint8_t>(1u << i);
    if (((e.coupledPairs | e.measuredPairs) & bit) == 0) continue;
    const int x = probeNodeWithRole(PROBE_PAIRS[i], NodeRole::Terminal, 0);
    const int y = probeNodeWithRole(PROBE_PAIRS[i], NodeRole::Terminal, 1);
    Serial.printf("precheck: %c-%c %s-%s %s", PROBE_NODE_LABEL[x], PROBE_NODE_LABEL[y],
                  probeRoleName(e.mapping.role[x], e.signalLabel), probeRoleName(e.mapping.role[y], e.signalLabel),
                  (e.coupledPairs & bit) ? "coupled" : "not-coupled");
    if (e.measuredPairs & bit) {
      const PairEvidence &p = e.pair[i];
      Serial.printf(" -> %s", couplingClassificationName(p.classification));
      if (p.classification != CouplingClassification::MeasurementError) {
        Serial.printf(" %u/%u mV", (unsigned)p.mv[0], (unsigned)p.mv[1]);
      }
    }
    Serial.println();
  }
  Serial.printf("candidate: %s; %s\n", allowed ? "allowed" : "blocked", e.reason);
}

// Non-blocking: handles only bytes that have already arrived.
//   r  retry after a latched fault
//   c  clear the result and rescan (e.g. a switch left OPEN: an open switch
//      cannot be told apart from removal)
//   i  print the detected driver's diagnostics (e.g. every I2C address)
//   other letters go to the detected driver (passive: l = guided LDR
//   calibration start/cancel, d = take dark reference, b = take bright)
static void handleSerialCommands() {
  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c == 'r' || c == 'R') {
      Serial.println(probeRequestRetry() ? "retry: fault cleared, rerunning precheck"
                                         : "retry: ignored (no latched fault, or probing disabled)");
    } else if (c == 'c' || c == 'C') {
      Serial.println(probeRequestClear() ? "clear: result cleared, rescanning"
                                         : "clear: ignored (probing disabled)");
    } else if (c == 'i' || c == 'I') {
      printDiagnostics();
    } else if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')) {
      Serial.printf("command '%c': %s\n", c,
                    probeSendCommand(static_cast<char>(c)) ? "accepted" : "ignored");
    }
  }
}

void setup() {
  initProbeEngine();  // First hardware action: probes to high impedance.

  Serial.begin(SERIAL_BAUD_RATE);
  Serial.print("PinDex v" PINDEX_VERSION " - drivers:");
  for (size_t i = 0; i < sensorDriverCount(); i++) {
    Serial.printf(" [%s]", sensorDriverAt(i)->getName());
  }
  Serial.println();
  Serial.println(PINDEX_POWERED_PROBING_ENABLED
                     ? "Powered probing ENABLED: requires the protected interface (README)."
                     : "Powered probing disabled: no probe is driven as VCC/GND.");
  Serial.println(PINDEX_PASSIVE_PROBING_ENABLED
                     ? "Passive probing ENABLED: weak pull-up + open-drain sink; unpowered parts only."
                     : "Passive probing disabled.");
  if (PINDEX_POWERED_PROBING_ENABLED && PINDEX_I2C_GPIO_POWER_ENABLED) {
    Serial.println("EXPERIMENTAL I2C GPIO power profile ENABLED: not protection (README).");
  }
  for (size_t i = 0; i < sensorDriverCount() && i < 8; i++) {
    if (probeStatus().hardwareRequiredMask & (1u << i)) {
      Serial.printf("%s: hardware required (no eligible supply profile); not attempted.\n",
                    sensorDriverAt(i)->getName());
    }
  }
  Serial.println("Serial: r=retry fault, c=clear/rescan, i=diagnostics, l/d/b=guided LDR calibration.");

  displayOk = initDisplay();
  if (displayOk) {
    showSplashScreen();
    splashStartMs = millis();
    splashPending = true;
  } else {
    resetTestPins();
    Serial.println("ERROR: OLED init failed; continuing without display.");
  }

  lastHeartbeatMs = millis();
}

void loop() {
  probeTick();
  handleSerialCommands();

  // Unsigned subtraction keeps every comparison correct across the
  // ~49.7-day millis() rollover.
  const unsigned long now = millis();

  const ProbeStatus &probe = probeStatus();
  if (probe.evaluation.serial != loggedEvaluation) {
    loggedEvaluation = probe.evaluation.serial;
    logEvaluation(probe.evaluation, now);
  }
  if (probe.inconclusiveCount > loggedInconclusive) {  // one line per skip; the status line follows
    Serial.printf("precheck: attempt %u/%u inconsistent; retries exhausted; skipping\n",
                  (unsigned)probe.inconclusiveAttempt, (unsigned)probe.scanTotal);
  }
  loggedInconclusive = probe.inconclusiveCount;
  if (probe.revision != loggedRevision) {
    loggedRevision = probe.revision;
    logProbeStatus(probe);
  }

  // The splash is drawn once in setup(). When it expires the screen follows
  // the probe status, so the splash can never overwrite a result or fault.
  if (splashPending) {
    if (now - splashStartMs >= SPLASH_DURATION_MS) {
      splashPending = false;
      shownRevision = probe.revision;
      showProbeStatus(probe);
    }
  } else if (probe.revision != shownRevision) {
    shownRevision = probe.revision;
    showProbeStatus(probe);  // no-op if the display failed to initialise
  }

  if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatMs = now;  // Re-anchor, so a stall never causes a burst.
    Serial.printf("heartbeat uptime=%lus display=%s probe=%s\n", now / 1000UL,
                  displayOk ? "ok" : "failed", probePhaseName(probe.phase));
  }
}
