// OLED screens for PinDex. All screen functions are no-ops if initDisplay()
// failed, so callers never need to guard them.
#pragma once

// Starts VSPI and the SSD1306. Returns true if the driver initialised.
// The SPI link is write-only, so true does NOT prove a panel is attached.
bool initDisplay();

// Draws the splash screen once and returns; the caller owns its timing.
void showSplashScreen();

// Draws the idle "Ready" screen once.
void showIdleScreen();
