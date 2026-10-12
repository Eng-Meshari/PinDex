// ESP32 implementation of the probe I2C adapter (Arduino Wire1).
//
// Checked against Arduino-ESP32 3.3.12 / ESP-IDF 5.5.5 (Wire.cpp and
// esp32-hal-i2c-ng.c, the driver used from IDF 5.4 on):
//   * TwoWire::begin() returns true WITHOUT reconfiguring anything when the
//     controller is already running, so i2cIsInit() is checked first; a
//     controller started elsewhere is reported as a conflict, never ended.
//   * i2cInit() sets SDA/SCL up open-drain with the internal pull-ups on
//     (flags.enable_internal_pullup). Neither line is ever push-pull HIGH.
//   * endTransmission(true) with no payload calls i2c_master_probe():
//     ESP_OK -> 0, ESP_ERR_NOT_FOUND (NACK) -> 2, ESP_ERR_TIMEOUT -> 5,
//     anything else -> 4. On an uninitialised bus i2cWrite() returns
//     ESP_FAIL -> 2 too, so a probe is only issued while i2cIsInit()
//     confirms this adapter's controller is running.
//   * end() -> i2cDeinit() deletes the controller and releases the pin
//     ownership, reports a failed deinit as false, and leaves the pads as
//     they were. They are then reset here: pulls off, plain digital INPUT.
//
// Wire1 (controller 1), not Wire: libraries default to Wire (Adafruit's
// I2C displays, for one), and the probe bus must never be shared. Nothing
// else in PinDex uses Wire or Wire1 (the OLED is SPI). No TwoWire object is
// created here: Wire1 already exists for controller 1, and a second object
// for the same controller would compete with it.

#include "i2c_bus.h"

#include <Arduino.h>
#include <Wire.h>

#include "driver/gpio.h"

#include "../../config.h"

namespace {

TwoWire &bus = Wire1;
constexpr uint8_t kController = 1;
constexpr uint8_t kNoPin = 0xFF;

bool owned = false;       // this adapter started the controller and has not stopped it
uint8_t sdaPin = kNoPin;  // pins of the current session, until released
uint8_t sclPin = kNoPin;

void pullsOff(uint8_t pin) {
  if (pin == kNoPin) return;
  gpio_pullup_dis(static_cast<gpio_num_t>(pin));
  gpio_pulldown_dis(static_cast<gpio_num_t>(pin));
}

// Pulls off and plain digital input on the session's pins, once.
void releasePins() {
  for (uint8_t pin : {sdaPin, sclPin}) {
    if (pin == kNoPin) continue;
    pullsOff(pin);
    pinMode(pin, INPUT);
  }
  sdaPin = sclPin = kNoPin;
}

}  // namespace

I2cBusStart i2cBusBegin(uint8_t sda, uint8_t scl) {
  if (owned || i2cIsInit(kController)) return I2cBusStart::Conflict;
  sdaPin = sda;
  sclPin = scl;
  if (!bus.begin(sda, scl, I2C_SCAN_FREQUENCY_HZ)) {
    bus.end();  // frees what a partial start kept; the controller never came up
    releasePins();
    return I2cBusStart::Failed;
  }
  owned = true;
  bus.setTimeOut(I2C_TRANSACTION_TIMEOUT_MS);
  return I2cBusStart::Ok;
}

bool i2cBusIdle() {
  return owned && gpio_get_level(static_cast<gpio_num_t>(sdaPin)) == 1 &&
         gpio_get_level(static_cast<gpio_num_t>(sclPin)) == 1;
}

I2cBusResult i2cBusProbe(uint8_t address) {
  if (!owned || !i2cIsInit(kController)) return I2cBusResult::BusError;  // never a "clean NACK"
  bus.beginTransmission(address);
  return i2cBusResultFromWire(bus.endTransmission(true));  // STOP; no repeated start left pending
}

bool i2cBusEnd() {
  if (owned) {
    if (!bus.end() || i2cIsInit(kController)) {
      // Still running: ownership uncertain. The pulls go off anyway
      // (directly on the pads); the pins stay with the controller.
      pullsOff(sdaPin);
      pullsOff(sclPin);
      return false;
    }
    owned = false;
  }
  releasePins();
  return true;
}
