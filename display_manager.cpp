#include "display_manager.h"

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "config.h"

namespace {

SPIClass oledSpi(VSPI);
Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &oledSpi, OLED_DC_PIN,
                      OLED_RESET_PIN, OLED_CS_PIN, OLED_SPI_FREQUENCY_HZ);
bool displayReady = false;

// Prints text horizontally centred with its top edge at row y.
void drawCentered(const char *text, uint8_t size, int16_t y) {
  int16_t x1, y1;
  uint16_t w, h;
  oled.setTextSize(size);
  oled.getTextBounds(text, 0, y, &x1, &y1, &w, &h);
  oled.setCursor((OLED_WIDTH - (int16_t)w) / 2 - x1, y);
  oled.print(text);
}

// Two thin lines, drawn as the "=====" border of the idle screen.
void drawDoubleRule(int16_t y) {
  oled.drawFastHLine(0, y, OLED_WIDTH, SSD1306_WHITE);
  oled.drawFastHLine(0, y + 2, OLED_WIDTH, SSD1306_WHITE);
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
  drawCentered("PinDex", 2, 16);                   // rows 16-31
  drawCentered("Firmware v" PINDEX_VERSION, 1, 40);  // rows 40-47
  oled.display();
}

void showIdleScreen() {
  if (!displayReady) return;

  // Rows 3-60 sit between the rules; the 32px of text is spread evenly.
  oled.clearDisplay();
  drawDoubleRule(0);                                 // rows 0-2
  drawCentered("PinDex", 2, 9);                      // rows 9-24
  drawCentered("v" PINDEX_VERSION " Ready", 1, 32);  // rows 32-39
  drawCentered("Insert Component...", 1, 46);        // rows 46-53
  drawDoubleRule(OLED_HEIGHT - 3);                   // rows 61-63
  oled.display();
}
