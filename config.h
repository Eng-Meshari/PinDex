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
// 0: no probe GPIO is ever driven as VCC/GND (passive weak-pull probing is
// controlled separately below; both 0 = probes stay plain INPUT and the
// display shows "Probing disabled"). 1: run the six-permutation powered
// scan, driving probe GPIOs as a nominal 3.3V supply where the precheck
// shows no coupling on the VCC node.
// Setting 1 is a DEPLOYMENT PREREQUISITE, NOT A SAFETY MECHANISM. Only enable
// it behind a reviewed interface with current limiting, fault handling and
// protection against wrong polarity and pin assignment (see README).
// The #ifndef only lets host tests override it with -D.
#ifndef PINDEX_POWERED_PROBING_ENABLED
#define PINDEX_POWERED_PROBING_ENABLED 1
#endif

// Passive weak-pull probing (Phase 3.2, src/drivers/passive_driver.cpp).
// Independent of PINDEX_POWERED_PROBING_ENABLED: one node's internal pull-up
// is sensed by the ADC while another node sinks LOW through an open-drain
// output; no probe node is ever driven HIGH. With BOTH flags 0 every probe
// stays plain INPUT (global disable). Connect only UNPOWERED parts: the LOW
// sink is not protected against an external voltage source.
#ifndef PINDEX_PASSIVE_PROBING_ENABLED
#define PINDEX_PASSIVE_PROBING_ENABLED 1
#endif

// Weak-pull precheck (diagnostic heuristic, not a safety test). It routes:
// a pulled-down node reading HIGH marks that pair as coupled, which blocks
// powered drive across it; inconsistent readings are retried, then "unknown".
#define PRECHECK_SETTLE_MS 5UL           // after changing pulls, before sample 1
#define PRECHECK_SAMPLE_INTERVAL_MS 2UL  // between samples
#define PRECHECK_SAMPLES 4               // per pull-up source node
#define PRECHECK_MAX_RETRIES 2           // extra passes after inconsistent readings

// Engine power policy (driver-independent). Stabilization and acquisition
// spacing come from each driver's DriverRequirements.
#define PROBE_POWER_OFF_MS 300UL         // between attempts; NOT proof of discharge
#define PROBE_RETRY_COOLDOWN_MS 1000UL   // after a failed read on the saved mapping
#define PROBE_RESCAN_INTERVAL_MS 5000UL  // idle time before discovery restarts
#define PROBE_CONFIRM_FRAMES 2           // consecutive valid frames to publish
#define PROBE_MAX_CONSECUTIVE_FAILURES 3 // then removal is inferred

// DHT11 driver (src/drivers/dht11_driver.cpp): requirements and protocol timing.
#define DHT_POWER_STABILIZE_MS 1100UL    // power-up to first start request
#define DHT_OPERATION_TIMEOUT_MS 500UL   // engine abandons a still-pending acquisition
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

// Passive driver: timing.
#define PASSIVE_SAMPLE_PERIOD_MS 100UL       // monitoring cadence, start to start
#define PASSIVE_OPERATION_TIMEOUT_MS 1000UL  // engine abandons a still-pending measurement
#define PASSIVE_SETTLE_MS 5UL                // after any excitation change
#define PASSIVE_SAMPLE_INTERVAL_MS 2UL       // between ADC samples in one window
#define PASSIVE_REFERENCE_SAMPLES 2          // pull-up check with the sink released
#define PASSIVE_WINDOW_SAMPLES 4             // divider samples per orientation

// Passive driver: divider model. Approximate by design: the internal
// pull-up is only nominally 45 kOhm (wide part-to-part and temperature
// spread) and the rail is nominally 3.3 V.
#define PASSIVE_SUPPLY_MV 3300               // rail feeding the pull-up (nominal)
#define PASSIVE_PULLUP_NOMINAL_OHMS 45000UL  // NOT a precision reference
#define PASSIVE_ADC_DEFAULT_VREF_MV 1100     // line-fitting calibration if eFuse Vref is absent
#define PASSIVE_ADC_UNCAL_FULL_SCALE_MV 3100 // raw 4095 if no calibration scheme is available
#define PASSIVE_PULLUP_VERIFY_MIN_MV 2450    // sink released: the pull-up must lift the node here
#define PASSIVE_CONTACT_MAX_MV 150           // at/below: Contact / Low R (useful-range floor)
#define PASSIVE_OPEN_MIN_MV 2450             // at/above: open-like / very high R (useful-range top)

// Passive driver: classification.
#define PASSIVE_STABLE_SPREAD_MV 80          // max spread inside one probe window
#define PASSIVE_SYMMETRY_MIN_DELTA_MV 60     // orientations may differ by this much...
#define PASSIVE_SYMMETRY_PCT 15              // ...or by this % of their mean
#define PASSIVE_DEBOUNCE_SAMPLES 2           // consecutive samples to accept OPEN/CLOSED
#define PASSIVE_OPEN_GRACE_SAMPLES 50        // resistive part open-like this long -> failure
#define PASSIVE_FILTER_SHIFT 2               // exponential filter weight 1/4
#define PASSIVE_VARIATION_MIN_MV 150         // filtered change counted as variation...
#define PASSIVE_VARIATION_CONFIRM_SAMPLES 5  // ...for this many consecutive samples
#define PASSIVE_LDR_WINDOW_SAMPLES 10        // guided dark/bright reference window
#define PASSIVE_LDR_MAX_WINDOW_SPREAD_MV 120 // each reference window must be this steady
#define PASSIVE_LDR_MIN_SEPARATION_MV 300    // dark minus bright needed to accept
#define PASSIVE_MESSAGE_SAMPLES 30           // samples a calibration message stays shown
