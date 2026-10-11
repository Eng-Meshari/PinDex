// PinDex - Smart Component & Sensor Identifier
// v0.1 firmware scaffold: OLED bring-up, non-blocking scheduling and an
// inactive probe engine. Component identification is NOT implemented.

#include "config.h"
#include "display_manager.h"
#include "probe_engine.h"

static bool displayOk = false;
static bool splashPending = false;  // true until the idle screen replaces it
static unsigned long splashStartMs = 0;
static unsigned long lastHeartbeatMs = 0;

void setup() {
  initProbeEngine();  // First hardware action: probes to high impedance.

  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println("PinDex v" PINDEX_VERSION
                 " scaffold - component identification not implemented.");

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

  // Unsigned subtraction keeps every comparison correct across the
  // ~49.7-day millis() rollover.
  const unsigned long now = millis();

  if (splashPending && now - splashStartMs >= SPLASH_DURATION_MS) {
    splashPending = false;
    showIdleScreen();
  }

  if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatMs = now;  // Re-anchor, so a stall never causes a burst.
    Serial.printf("heartbeat uptime=%lus display=%s\n", now / 1000UL,
                  displayOk ? "ok" : "failed");
  }
}
