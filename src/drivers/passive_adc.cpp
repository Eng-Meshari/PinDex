// ESP32 implementation of the passive ADC adapter (ESP-IDF oneshot ADC).
//
// Checked against the installed Arduino-ESP32 3.3.12 / ESP-IDF 5.5.5
// libraries (relocations in libesp_adc.a / libesp_driver_gpio.a):
//   * adc_oneshot_config_channel() calls gpio_config_as_analog(), which
//     calls gpio_pullup_dis()/gpio_pulldown_dis(): channel setup REMOVES the
//     pull-up, so pinMode(INPUT_PULLUP) + analogRead() cannot keep a divider.
//   * adc_oneshot_read() does not reconfigure the pad.
//   * gpio_pullup_en() on an RTC-capable pad calls rtc_gpio_pullup_en()
//     (on the ESP32 these pads' pulls live in the RTC IO registers).
//   * gpio_config() (Arduino pinMode) calls rtc_gpio_deinit(), enables the
//     input and sets pulls, so pinMode(INPUT) restores a digital input.
// Hence: configure the channel, THEN enable the pull-up; read with error
// checking; restore with pulls off + pinMode(INPUT). Arduino analogRead()
// is not used: it discards adc_oneshot_read() errors (esp32-hal-adc.c), so
// a failed conversion would look like a valid 0 mV, and mixing it with this
// adapter would create two owners of one ADC unit.
//
// Whether the pull-up is electrically effective while the pad is sampled
// is checked at run time by PassiveDriver (it must lift a node that is not
// sunk to near the rail); it has not been confirmed on hardware.
//
// GPIO25/26/27 are ADC2 on the original ESP32. ADC2 conflicts (e.g. with
// Wi-Fi, which PinDex does not use) surface as Unavailable/ReadFailed.

#include "passive_adc.h"

#include <Arduino.h>

#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

#include "../../config.h"

namespace {

constexpr adc_atten_t kAtten = ADC_ATTEN_DB_12;  // ~0-3.1 V span; linear ~0.15-2.45 V
constexpr int kRawFullScale = 4095;              // 12-bit

adc_oneshot_unit_handle_t unitHandle = nullptr;
adc_unit_t unitId = ADC_UNIT_2;
bool unitAttempted = false;
adc_cali_handle_t caliHandle = nullptr;

bool channelFor(uint8_t pin, adc_unit_t &unit, adc_channel_t &channel) {
  return adc_oneshot_io_to_channel(pin, &unit, &channel) == ESP_OK;
}

// Claims the ADC unit once for the firmware's lifetime: one allocation at
// first use, never repeated while probing.
bool ensureUnit(adc_unit_t unit) {
  if (unitHandle != nullptr) return unit == unitId;
  if (unitAttempted) return false;
  unitAttempted = true;

  adc_oneshot_unit_init_cfg_t init = {};
  init.unit_id = unit;
  init.ulp_mode = ADC_ULP_MODE_DISABLE;
  if (adc_oneshot_new_unit(&init, &unitHandle) != ESP_OK) {
    unitHandle = nullptr;
    return false;
  }
  unitId = unit;

  adc_cali_line_fitting_config_t cali = {};
  cali.unit_id = unit;
  cali.atten = kAtten;
  cali.bitwidth = ADC_BITWIDTH_12;
  cali.default_vref = PASSIVE_ADC_DEFAULT_VREF_MV;  // used only without eFuse data
  if (adc_cali_create_scheme_line_fitting(&cali, &caliHandle) != ESP_OK) caliHandle = nullptr;
  return true;
}

}  // namespace

PassiveAdcStatus passiveAdcBeginSense(uint8_t pin) {
  adc_unit_t unit;
  adc_channel_t channel;
  if (!channelFor(pin, unit, channel)) return PassiveAdcStatus::ConfigFailed;
  if (!ensureUnit(unit)) return PassiveAdcStatus::Unavailable;

  adc_oneshot_chan_cfg_t config = {};
  config.atten = kAtten;
  config.bitwidth = ADC_BITWIDTH_12;
  if (adc_oneshot_config_channel(unitHandle, channel, &config) != ESP_OK) {
    return PassiveAdcStatus::ConfigFailed;
  }
  // After channel setup (which disabled pulls): weak pull-up only.
  const gpio_num_t gpioNum = static_cast<gpio_num_t>(pin);
  if (gpio_pulldown_dis(gpioNum) != ESP_OK || gpio_pullup_en(gpioNum) != ESP_OK) {
    return PassiveAdcStatus::ConfigFailed;
  }
  return PassiveAdcStatus::Ok;
}

PassiveAdcStatus passiveAdcReadMv(uint8_t pin, uint16_t &millivolts) {
  adc_unit_t unit;
  adc_channel_t channel;
  if (unitHandle == nullptr || !channelFor(pin, unit, channel) || unit != unitId) {
    return PassiveAdcStatus::ConfigFailed;
  }
  int raw = 0;
  if (adc_oneshot_read(unitHandle, channel, &raw) != ESP_OK) {
    return PassiveAdcStatus::ReadFailed;  // e.g. ESP_ERR_TIMEOUT: result invalid
  }
  int mv = 0;
  if (caliHandle != nullptr) {
    if (adc_cali_raw_to_voltage(caliHandle, raw, &mv) != ESP_OK) return PassiveAdcStatus::ReadFailed;
  } else {
    mv = raw * PASSIVE_ADC_UNCAL_FULL_SCALE_MV / kRawFullScale;  // nominal, uncalibrated
  }
  if (mv < 0) mv = 0;
  if (mv > 0xFFFF) mv = 0xFFFF;
  millivolts = static_cast<uint16_t>(mv);
  return PassiveAdcStatus::Ok;
}

void passiveAdcEndSense(uint8_t pin) {
  const gpio_num_t gpioNum = static_cast<gpio_num_t>(pin);
  gpio_pullup_dis(gpioNum);
  gpio_pulldown_dis(gpioNum);
  pinMode(pin, INPUT);  // digital IO MUX, input enabled, no pulls
}

bool passiveAdcCalibrated() { return caliHandle != nullptr; }
