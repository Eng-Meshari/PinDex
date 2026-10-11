// Hardware adapter for PassiveDriver: weak-pull ADC sensing on one probe
// node. The only code that touches the ADC. The ESP32 build links
// passive_adc.cpp (ESP-IDF oneshot ADC); host tests link a simulated
// implementation of these functions instead.
#pragma once

#include <stdint.h>

enum class PassiveAdcStatus : uint8_t {
  Ok,
  Unavailable,   // ADC unit could not be claimed (e.g. ADC2 already owned)
  ConfigFailed,  // not an ADC pin, or channel/pull setup rejected
  ReadFailed,    // conversion returned an error; the result is invalid
};

// Routes `pin` to its ADC channel, then enables its internal pull-up (the
// channel setup itself disables pulls). Touches no other pin. On failure
// the caller must still call passiveAdcEndSense(pin).
PassiveAdcStatus passiveAdcBeginSense(uint8_t pin);

// One conversion in millivolts (calibrated when possible). `millivolts` is
// written only on Ok, so a failed read is never mistaken for 0 mV.
PassiveAdcStatus passiveAdcReadMv(uint8_t pin, uint16_t &millivolts);

// Pull-up and pull-down off, pad back to a plain digital INPUT usable by
// any other driver. Idempotent; never energises the pin.
void passiveAdcEndSense(uint8_t pin);

// Whether millivolts come from an ADC calibration scheme (else nominal).
bool passiveAdcCalibrated();
