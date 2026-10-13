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

// Probe socket: four generic test nodes (not fixed VCC/GND/signal
// terminals) in external socket order A, B, C, D. These are GPIO labels;
// they are not in this order, or adjacent, on every DevKit header. The
// node table in probe_engine.cpp is the only node-to-GPIO mapping.
// GPIO25/26/27 are ADC2 channels; GPIO33 is ADC1 (passive_adc.cpp).
#define PROBE_PIN_A 25
#define PROBE_PIN_B 26
#define PROBE_PIN_C 27
#define PROBE_PIN_D 33
#define PROBE_NODE_COUNT 4

#define HEARTBEAT_INTERVAL_MS 1000UL
#define SPLASH_DURATION_MS 1500UL

#define PINDEX_VERSION "0.2"

// ---------------------------------------------------------------------------
// Powered probing (three-wire DHT11, four-wire I2C and ultrasonic detection)
// ---------------------------------------------------------------------------
// 0: nothing is ever powered and no probe GPIO is driven as VCC/GND, a
// trigger or a bus (passive weak-pull probing is controlled separately
// below; both 0 = probes stay plain INPUT and the display shows "Probing
// disabled"). It also overrides PINDEX_I2C_GPIO_POWER_ENABLED.
// 1: run the powered scans:
//   * three-wire (24 mappings): probe GPIOs driven as a nominal 3.3V
//     supply where the precheck shows no coupling on the VCC node;
//   * four-wire (24 mappings): ONLY with an eligible supply. The protected
//     four-wire backend (four_wire_backend.h) must report the capabilities
//     the driver needs (the shipped one reports none), or, for the I2C
//     driver only, the experimental GPIO profile below must be enabled.
//     This setting cannot enable either on its own.
// Setting 1 is a DEPLOYMENT PREREQUISITE, NOT A SAFETY MECHANISM. Only enable
// it behind a reviewed interface with current limiting, fault handling and
// protection against wrong polarity and pin assignment (see README).
// The #ifndef only lets host tests override it with -D.
#ifndef PINDEX_POWERED_PROBING_ENABLED
#define PINDEX_POWERED_PROBING_ENABLED 1
#endif

// EXPERIMENTAL, default 0. 1: when no protected 3.3V backend exists, power
// the I2C driver's four-wire mappings straight from probe GPIOs (candidate
// GND driven LOW, VCC driven HIGH: a nominal 3.3V logic level, not a
// regulated rail). Every precheck veto still applies, and
// PINDEX_POWERED_PROBING_ENABLED 0 still disables it. This is NOT
// electrical protection: software limits no fault current and prevents no
// reverse-polarity damage, and many breakouts (regulators, LEDs, LCD
// backlights, pull-ups, inrush) need more than a GPIO can source. Enable it
// only for a module whose supply needs and tolerance of wrong assignments
// you have evaluated (README). Never enable it because an address hint
// suggests a low-current part. It never powers the 5V ultrasonic driver.
#ifndef PINDEX_I2C_GPIO_POWER_ENABLED
#define PINDEX_I2C_GPIO_POWER_ENABLED 1
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
// powered drive across it. Readings that change within a pass are retried;
// if they never settle, only that attempt is skipped (never powered) and
// discovery goes on. The waits let charge left by the previous attempt
// (e.g. a breakout's capacitors) bleed through the pulls. Provisional
// values, NOT validated on hardware; no wait proves a capacitor discharged.
#define PRECHECK_SETTLE_MS 20UL          // after each pull change, before sample 1
#define PRECHECK_SAMPLE_INTERVAL_MS 5UL  // between samples under one pull configuration
#define PRECHECK_SAMPLES 4               // per pull-up source node
#define PRECHECK_MAX_RETRIES 2           // extra passes after inconsistent readings (1 + 2 passes)

// Narrow open-drain pull-up exception (probe_engine.cpp). Only for a
// FourWirePowered + Supply3V3 + open-drain candidate (the I2C scanner)
// whose supply profile is already eligible, and only when the precheck saw
// its VCC node coupled to SIG1/SIG2 and to nothing else. Then VCC-SIG1,
// VCC-SIG2 and SIG1-SIG2 are each measured with the passive weak-pull
// meter (internal pull-up + open-drain LOW sink, supply off, both
// orientations). Bands are sensed mV of the nominal 45 kOhm pull-up
// divider, derived on paper and NOT validated on hardware:
//   4.7k-10k (+-5%) against an assumed 30k-67k internal pull-up spread
//   gives ~206-856 mV; ~+-40 mV ADC uncertainty widens it to 170-900 mV.
//   At or below PASSIVE_CONTACT_MAX_MV (150 mV, near the ADC's floor at
//   12 dB) a path cannot be told from a short: low impedance, never allowed.
//   SIG1-SIG2 must match the two pull-ups in series (common VCC) within
//   PULLUP_SERIES_TOLERANCE_PCT, using the same approximate estimate as the
//   passive driver; pin-to-pin pull-up mismatch and ADC error dominate it.
// Stability and symmetry reuse PASSIVE_STABLE_SPREAD_MV and
// PASSIVE_SYMMETRY_*. Narrowing these bands only makes the policy stricter.
#define PULLUP_CLASSIFY_SETTLE_MS 20UL   // after each excitation change (module capacitance)
#define PULLUP_COMPATIBLE_MIN_MV 170     // sensed mV band of one supported pull-up
#define PULLUP_COMPATIBLE_MAX_MV 900
#define PULLUP_SERIES_TOLERANCE_PCT 35   // SIG1-SIG2 vs. the sum of both pull-up estimates
#define PRECHECK_LOG_INTERVAL_MS 1000UL  // Serial: at most one blocked evaluation per interval

// Engine power policy (driver-independent). Stabilization and acquisition
// spacing come from each driver's DriverRequirements.
#define PROBE_POWER_OFF_MS 300UL         // between attempts; NOT proof of discharge
#define PROBE_RETRY_COOLDOWN_MS 1000UL   // after a failed read on the saved mapping
#define PROBE_RESCAN_INTERVAL_MS 5000UL  // idle time before discovery restarts
#define PROBE_CONFIRM_FRAMES 2           // consecutive valid frames to publish
#define PROBE_MAX_CONSECUTIVE_FAILURES 3 // then loss of response (e.g. removal) is inferred

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

// HC-SR04-compatible ultrasonic driver (src/drivers/ultrasonic_driver.cpp).
// Runs only behind a protected four-wire backend (see README): the module
// needs a 5V supply and returns a 5V ECHO.
#define ULTRASONIC_POWER_STABILIZE_MS 200UL   // backend power-on to first trigger. An
                                              // engineering choice, NOT a datasheet figure:
                                              // validate on the actual module
#define ULTRASONIC_TRIGGER_PULSE_US 10UL      // TRIG HIGH width (HC-SR04 needs >= 10 us)
#define ULTRASONIC_ECHO_DEADLINE_US 25000UL   // trigger release to ECHO falling edge, one budget
#define ULTRASONIC_MIN_INTERVAL_MS 70UL       // trigger to trigger, incl. retries
#define ULTRASONIC_SAMPLE_INTERVAL_MS 100UL   // monitoring cadence, start to start
#define ULTRASONIC_OPERATION_TIMEOUT_MS 300UL // engine abandons a still-pending operation
#define ULTRASONIC_CONFIRM_SAMPLES 2          // consecutive fresh valid echoes per probe()
#define ULTRASONIC_US_PER_CM 58.0f            // round-trip echo time per cm
#define ULTRASONIC_MIN_DISTANCE_CM 2.0f       // accepted range, inclusive
#define ULTRASONIC_MAX_DISTANCE_CM 300.0f

// I2C scanner driver (src/drivers/i2c_driver.cpp, bus adapter i2c_bus.cpp).
#define I2C_POWER_STABILIZE_MS 100UL      // supply on to bus start. An engineering choice, NOT a
                                          // datasheet figure: validate on the actual module
#define I2C_SCAN_FREQUENCY_HZ 100000UL    // standard mode
#define I2C_TRANSACTION_TIMEOUT_MS 25     // Wire setTimeOut(): bounds one transaction (plus overhead)
#define I2C_IDLE_SETTLE_MS 10UL           // SDA and SCL must read HIGH within this, before every
                                          // transaction and before a result is accepted
#define I2C_SWEEP_DEADLINE_MS 1000UL      // one whole 0x08-0x77 sweep, first to last address
#define I2C_MAX_SWEEPS 3                  // per probe(): two consecutive sweeps must agree
#define I2C_SAMPLE_INTERVAL_MS 2000UL     // liveness check and address rotation, start to start
#define I2C_OPERATION_TIMEOUT_MS 3500UL   // engine abandons a still-pending operation
#define I2C_FIRST_ADDRESS 0x08            // scan range, inclusive (0x00-0x07 and 0x78-0x7F are
#define I2C_LAST_ADDRESS 0x77             // reserved and never addressed)

// Binary digital sensor driver (src/drivers/binary_driver.cpp): the generic
// discovery profile. Provisional engineering choices, NOT validated on
// hardware or against any module's datasheet. A module that needs a longer
// warm-up (e.g. a PIR) is verified in the guided mode after waiting, rather
// than by lengthening every one of the 24 discovery attempts.
#define BINARY_WARMUP_MS 100UL           // OUT bias applied -> first counted sample
#define BINARY_SAMPLE_INTERVAL_MS 10UL   // probe sampling and monitoring cadence
#define BINARY_STABLE_MS 200UL           // a level must hold this long before it is a candidate
#define BINARY_DEBOUNCE_MS 20UL          // monitoring: a new level is shown after holding this long
#define BINARY_MAX_OBSERVE_MS 1000UL     // per attempt, after warm-up: no stable level -> inconclusive
#define BINARY_MAX_TRANSITIONS 4         // more changes in one observation: toggling, inconclusive
#define BINARY_BIAS_SETTLE_MS 10UL       // floating check: after each pull change, before reading
#define BINARY_VERIFY_WINDOW_MS 500UL    // guided: stable level needed after each mark
#define BINARY_VERIFY_TIMEOUT_MS 5000UL  // guided: that window must complete within this
#define BINARY_MESSAGE_MS 3000UL         // guided: a result message stays shown this long

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
