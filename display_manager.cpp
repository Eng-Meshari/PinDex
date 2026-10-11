#include "display_manager.h"

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <stdio.h>

#include "config.h"

namespace {

SPIClass oledSpi(VSPI);
Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &oledSpi, OLED_DC_PIN,
                      OLED_RESET_PIN, OLED_CS_PIN, OLED_SPI_FREQUENCY_HZ);
bool displayReady = false;

const char kReadyLine[] = "v" PINDEX_VERSION " Ready";

// Prints text horizontally centred with its top edge at row y. Text wider
// than the panel is left-aligned (clipped) instead of starting off-screen.
void drawCentered(const char *text, uint8_t size, int16_t y) {
  int16_t x1, y1;
  uint16_t w, h;
  oled.setTextSize(size);
  oled.getTextBounds(text, 0, y, &x1, &y1, &w, &h);
  int16_t x = (OLED_WIDTH - (int16_t)w) / 2 - x1;
  if (x < 0) x = 0;
  oled.setCursor(x, y);
  oled.print(text);
}

// Two thin lines, drawn as the "=====" border of the idle screen.
void drawDoubleRule(int16_t y) {
  oled.drawFastHLine(0, y, OLED_WIDTH, SSD1306_WHITE);
  oled.drawFastHLine(0, y + 2, OLED_WIDTH, SSD1306_WHITE);
}

// Bordered "PinDex" screen with two status lines (idle layout).
void drawStatusScreen(const char *line1, const char *line2) {
  // Rows 3-60 sit between the rules; the 32px of text is spread evenly.
  oled.clearDisplay();
  drawDoubleRule(0);                // rows 0-2
  drawCentered("PinDex", 2, 9);     // rows 9-24
  drawCentered(line1, 1, 32);       // rows 32-39
  drawCentered(line2, 1, 46);       // rows 46-53
  drawDoubleRule(OLED_HEIGHT - 3);  // rows 61-63
  oled.display();
}

// Detected-component result from the engine snapshot: driver name, the
// driver's telemetry lines and the mapping. No sensor-specific text here.
// All lines are size 1 (21 glyphs max); the mapping is the widest at 126px.
void drawResultScreen(const ProbeStatus &s) {
  char line[32];
  const bool twoLines = s.telemetry2[0] != '\0';
  oled.clearDisplay();
  drawCentered(s.componentName != nullptr ? s.componentName : "", 1, 2);  // rows 2-9
  oled.drawFastHLine(0, 12, OLED_WIDTH, SSD1306_WHITE);

  if (twoLines) {
    drawCentered(s.telemetry1, 1, 16);  // rows 16-23
    drawCentered(s.telemetry2, 1, 26);  // rows 26-33
  } else {
    drawCentered(s.telemetry1, 1, 20);  // rows 20-27
  }

  formatProbeMapping(s.mapping, line, sizeof line);
  drawCentered(line, 1, twoLines ? 37 : 34);  // rows 37-44 or 34-41

  if (s.readingStale) {
    // Inverted bar so old values are never mistaken for live ones.
    oled.fillRect(0, 48, OLED_WIDTH, 16, SSD1306_WHITE);
    oled.setTextColor(SSD1306_BLACK);
    if (s.consecutiveFailures > 0) {
      snprintf(line, sizeof line, "STALE - fail %u/%u", (unsigned)s.consecutiveFailures,
               (unsigned)PROBE_MAX_CONSECUTIVE_FAILURES);
    } else {
      snprintf(line, sizeof line, "STALE");  // reconnected, awaiting a new sample
    }
    drawCentered(line, 1, 52);  // rows 52-59
    oled.setTextColor(SSD1306_WHITE);
  } else {
    drawCentered("LIVE", 1, 52);
  }
  oled.display();
}

}  // namespace

bool initDisplay() {
  // MISO is -1: the SSD1306 has no data output. CS is driven by the
  // Adafruit driver in software; it is passed here for completeness.
  // begin(..., periphBegin=false) stops the library re-initialising SPI.
  displayReady =
      oledSpi.begin(OLED_SCK_PIN, -1, OLED_MOSI_PIN, OLED_CS_PIN) &&
      oled.begin(SSD1306_SWITCHCAPVCC, 0, true, false);

  if (displayReady) {
    oled.setTextWrap(false);
    oled.setTextColor(SSD1306_WHITE);
  }
  return displayReady;
}

void showSplashScreen() {
  if (!displayReady) return;

  oled.clearDisplay();
  drawCentered("PinDex", 2, 16);                     // rows 16-31
  drawCentered("Firmware v" PINDEX_VERSION, 1, 40);  // rows 40-47
  oled.display();
}

void showIdleScreen() {
  if (!displayReady) return;
  drawStatusScreen(kReadyLine, "Insert Component...");
}

void showProbeStatus(const ProbeStatus &s) {
  if (!displayReady) return;

  char line[24];
  switch (s.phase) {
    case ProbePhase::Detected:
    case ProbePhase::Retrying:
      drawResultScreen(s);
      break;
    case ProbePhase::Disabled:
      drawStatusScreen(kReadyLine, "Probing disabled");
      break;
    case ProbePhase::Scanning:
      snprintf(line, sizeof line, "Scanning %u/%u", (unsigned)s.scanIndex,
               (unsigned)PROBE_PERMUTATION_COUNT);
      drawStatusScreen(line, "Insert Component...");
      break;
    case ProbePhase::Confirming:
      drawStatusScreen(s.componentName != nullptr ? s.componentName : "Candidate",
                       "Confirming...");
      break;
    case ProbePhase::NoComponent:
      drawStatusScreen("No Component /", "Unknown");
      break;
    case ProbePhase::Fault:
      drawStatusScreen("Socket rejected", "Send 'r' to retry");
      break;
    case ProbePhase::Idle:
    default:
      showIdleScreen();  // also restores "Insert Component..." after removal
      break;
  }
}
