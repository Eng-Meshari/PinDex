// PinDex central configuration. GPIO numbers are ESP32 GPIO labels,
// not physical header positions. Target: original ESP32 (WROOM-32) only.
#pragma once

#define SERIAL_BAUD_RATE 115200

// SSD1306 128x64 OLED on hardware VSPI (write-only, no MISO).
#define OLED_SCK_PIN 18
#define OLED_MOSI_PIN 23
#define OLED_RESET_PIN 4
#define OLED_DC_PIN 2  // Strapping pin: must not be held high during reset.
#define OLED_CS_PIN 5  // Strapping pin: must not be held low during reset.
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_SPI_FREQUENCY_HZ 8000000UL

// Probe test nodes (not fixed VCC/GND/signal terminals).
#define PROBE_PIN_A 25
#define PROBE_PIN_B 26
#define PROBE_PIN_C 27

#define HEARTBEAT_INTERVAL_MS 1000UL
#define SPLASH_DURATION_MS 1500UL

#define PINDEX_VERSION "0.1"
