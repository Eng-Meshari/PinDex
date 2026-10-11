// PinDex - Smart Component & Sensor Identifier
// Sensor drivers (src/drivers/) behind a generic probe engine: a
// DHT11-compatible powered driver and a weak-pull passive/switch driver.
// Powered and passive probing are controlled separately in config.h
// (PINDEX_POWERED_PROBING_ENABLED, PINDEX_PASSIVE_PROBING_ENABLED); read
// the README before enabling either.

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

static void logProbeStatus(const ProbeStatus &s) {
  Serial.printf("probe: %s", probePhaseName(s.phase));
  if (s.phase == ProbePhase::Scanning) {
    Serial.printf(" %u/%u", (unsigned)s.scanIndex, (unsigned)s.scanTotal);
  }
  if (s.precheck == PrecheckResult::CouplingObserved) {
    Serial.printf(" coupled=%s%s%s", (s.coupledPairs & 1) ? "AB " : "",
                  (s.coupledPairs & 2) ? "BC " : "", (s.coupledPairs & 4) ? "AC" : "");
  }
  if (s.componentName != nullptr) Serial.printf(" %s", s.componentName);
  if (s.hasMapping) {
    char mapping[PROBE_MAPPING_TEXT_LEN];
    formatProbeMapping(s.mapping, mapping, sizeof mapping);
    Serial.printf(" [%s]", mapping);
  }
  if (s.readingValid) {
    Serial.printf(" \"%s\"", s.telemetry1);
    if (s.telemetry2[0] != '\0') Serial.printf(" \"%s\"", s.telemetry2);
    if (s.readingStale) Serial.print(" STALE");
  }
  if (s.consecutiveFailures > 0) Serial.printf(" fails=%u", (unsigned)s.consecutiveFailures);
  if (s.lastError != nullptr) Serial.printf(" err=%s", s.lastError);
  if (s.measurementUnavailable) Serial.print(" measurement-unavailable");
  if (s.phase == ProbePhase::Fault) {
    Serial.printf(" precheck=%s; send 'r' to retry", precheckResultName(s.precheck));
  }
  Serial.println();
}

// Non-blocking: handles only bytes that have already arrived.
//   r  retry after a latched fault
//   c  clear the result and rescan (e.g. a switch left OPEN: an open switch
//      cannot be told apart from removal)
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
  Serial.println("Serial: r=retry fault, c=clear/rescan, l/d/b=guided LDR calibration.");

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

  const ProbeStatus &probe = probeStatus();
  if (probe.revision != loggedRevision) {
    loggedRevision = probe.revision;
    logProbeStatus(probe);
  }

  // Unsigned subtraction keeps every comparison correct across the
  // ~49.7-day millis() rollover.
  const unsigned long now = millis();

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
