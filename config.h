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

#define PINDEX_VERSION "0.2"

// ---------------------------------------------------------------------------
// Powered probing (Phase 2: DHT11-compatible detection)
// ---------------------------------------------------------------------------
// 0 (default): probes stay plain INPUT and the display shows
// "Probing disabled". 1: run the weak-pull precheck and the six-permutation
// powered scan, driving probe GPIOs as a nominal 3.3V supply.
// Setting 1 is a DEPLOYMENT PREREQUISITE, NOT A SAFETY MECHANISM. Only enable
// it behind a reviewed interface with current limiting, fault handling and
// protection against wrong polarity and pin assignment (see README).
// The #ifndef only lets host tests override it with -D.
#ifndef PINDEX_POWERED_PROBING_ENABLED
#define PINDEX_POWERED_PROBING_ENABLED 0
#endif

// Weak-pull precheck (diagnostic heuristic, not a safety test).
#define PRECHECK_SETTLE_MS 5UL           // after changing pulls, before sample 1
#define PRECHECK_SAMPLE_INTERVAL_MS 2UL  // between samples
#define PRECHECK_SAMPLES 4               // per pull-up source node

// Power sequencing.
#define PROBE_POWER_STABILIZE_MS 1100UL  // power-up to first start request
#define PROBE_POWER_OFF_MS 300UL         // between permutations; NOT proof of discharge
#define PROBE_RETRY_COOLDOWN_MS 1000UL   // after a failed read on the saved mapping
#define PROBE_RESCAN_INTERVAL_MS 5000UL  // idle time before discovery restarts
#define PROBE_CONFIRM_FRAMES 2           // consecutive valid frames to publish
#define PROBE_MAX_CONSECUTIVE_FAILURES 3 // then removal is inferred

// DHT single-wire protocol timing.
#define DHT_START_LOW_MS 20UL            // host start pulse (DHT11 needs >= 18 ms)
#define DHT_READ_INTERVAL_MS 2100UL      // start-to-start: confirmation and monitoring
#define DHT_MIN_START_INTERVAL_MS 2000UL // spacing on the saved mapping, incl. retries
#define DHT_PHASE_TIMEOUT_US 120UL       // any single LOW or HIGH phase
#define DHT_FRAME_DEADLINE_US 6500UL     // release to final bit, whole response
#define DHT_ACK_MIN_US 40                // acknowledgement LOW/HIGH, nominal 80 us
#define DHT_ACK_MAX_US 110
#define DHT_BIT_LOW_MIN_US 30            // bit LOW phase, nominal 50 us
#define DHT_BIT_LOW_MAX_US 80
#define DHT_BIT_HIGH_MIN_US 10           // bit HIGH phase: ~27 us = 0, ~70 us = 1
#define DHT_BIT_HIGH_MAX_US 100
#define DHT_BIT_THRESHOLD_US 50          // HIGH longer than this decodes as 1

// Supported DHT11 profile: plausibility window for decoded values.
#define DHT11_HUMIDITY_MIN_PCT 5
#define DHT11_HUMIDITY_MAX_PCT 95
#define DHT11_TEMPERATURE_MIN_C 0
#define DHT11_TEMPERATURE_MAX_C 50
