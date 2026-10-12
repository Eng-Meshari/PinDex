// Hardware adapter for I2cDriver: the probe I2C controller. The only code
// that touches that controller. The ESP32 build links i2c_bus.cpp (Wire1);
// host tests link a simulated bus instead.
#pragma once

#include <stdint.h>

enum class I2cBusStart : uint8_t {
  Ok,
  Conflict,  // the controller is already running (another module, or a stop
             // that failed): nothing was touched
  Failed,    // start rejected; anything it acquired was released again
};

enum class I2cBusResult : uint8_t {
  Ack,       // the address acknowledged
  Nack,      // an ordinary address miss on a healthy bus
  Timeout,   // the transaction timed out (stuck line, missing pull-ups, endless stretching)
  BusError,  // any other failure, or the controller is not running: bus state unknown
};

// Starts the controller on these pins (open-drain, internal pull-ups on) at
// I2C_SCAN_FREQUENCY_HZ with an I2C_TRANSACTION_TIMEOUT_MS timeout.
I2cBusStart i2cBusBegin(uint8_t sdaPin, uint8_t sclPin);

// Both lines read HIGH (released) right now. False when not started.
bool i2cBusIdle();

// One address-only write ending in STOP; no payload byte is sent.
// Synchronous: bounded by the transaction timeout plus driver overhead, not
// a hard real-time bound.
I2cBusResult i2cBusProbe(uint8_t address);

// Stops the controller if this adapter started it, then turns both pins'
// pulls off and returns them to plain digital INPUT. Never stops a
// controller it did not start. Idempotent. False if the controller would
// not stop (ownership uncertain; the pins are then left to it).
bool i2cBusEnd();

// TwoWire::endTransmission() codes for an address-only write on
// Arduino-ESP32 3.3.12: 0 = ACK; 2 = address NACK (ESP_ERR_NOT_FOUND; the
// core also returns 2 for ESP_FAIL on an uninitialised bus, which
// i2cBusProbe() rules out first); 5 = timeout; 4 = other error; 1 and 3
// need a payload and cannot occur. Only 2 is an ordinary miss.
inline I2cBusResult i2cBusResultFromWire(uint8_t code) {
  switch (code) {
    case 0: return I2cBusResult::Ack;
    case 2: return I2cBusResult::Nack;
    case 5: return I2cBusResult::Timeout;
    default: return I2cBusResult::BusError;
  }
}
