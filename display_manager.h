// OLED screens for PinDex. All screen functions are no-ops if initDisplay()
// failed, so callers never need to guard them.
#pragma once

#include "probe_engine.h"

// Starts VSPI and the SSD1306. Returns true if the driver initialised.
// The SPI link is write-only, so true does NOT prove a panel is attached.
bool initDisplay();

// Draws the splash screen once and returns; the caller owns its timing.
void showSplashScreen();

// Draws the idle "Ready / Insert Component..." screen once.
void showIdleScreen();

// Draws the screen for the current probe status: idle, scanning, fault,
// "No Component / Unknown", or the DHT11 result (marked STALE on retries).
// Call only when status.revision changes; never during a capture.
void showProbeStatus(const ProbeStatus &status);
