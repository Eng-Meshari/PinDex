// PinDex - Smart Component & Sensor Identifier
// Sensor drivers (src/drivers/) behind a generic probe engine; currently a
// DHT11-compatible driver with A/B/C pinout and live telemetry.
// Powered probing is controlled by PINDEX_POWERED_PROBING_ENABLED in
// config.h; read the README before enabling it.

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
    Serial.printf(" %u/%u", (unsigned)s.scanIndex, (unsigned)PROBE_PERMUTATION_COUNT);
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
  if (s.phase == ProbePhase::Fault) {
    Serial.printf(" precheck=%s; send 'r' to retry", precheckResultName(s.precheck));
  }
  Serial.println();
}

// Non-blocking: handles only bytes that have already arrived.
static void handleSerialCommands() {
  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c == 'r' || c == 'R') {
      Serial.println(probeRequestRetry() ? "retry: fault cleared, rerunning precheck"
                                         : "retry: ignored (no latched fault, or probing disabled)");
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
                     : "Powered probing disabled: probes stay high impedance.");

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
