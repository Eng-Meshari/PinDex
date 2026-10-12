// The only place that knows concrete drivers. To add one: implement a
// SensorDriver in src/drivers/, include it here, define one static instance
// and add it to the table. Priority: the engine tries the active protocols
// first (three-wire, then four-wire), then two-terminal (passive)
// interpretation; earlier entries go first within a topology, among the
// drivers whose supply is eligible. This table keeps that order: DHT11,
// I2C, ultrasonic, passive.
#include "driver_registry.h"

#include "dht11_driver.h"
#include "i2c_driver.h"
#include "passive_driver.h"
#include "ultrasonic_driver.h"

namespace {

// Live for the firmware's lifetime; never allocated.
Dht11Driver dht11Driver;
I2cDriver i2cDriver;
UltrasonicDriver ultrasonicDriver;
PassiveDriver passiveDriver;

SensorDriver *const registeredDrivers[] = {
    &dht11Driver,
    &i2cDriver,
    &ultrasonicDriver,
    &passiveDriver,
};

}  // namespace

size_t sensorDriverCount() { return sizeof registeredDrivers / sizeof registeredDrivers[0]; }

SensorDriver *sensorDriverAt(size_t index) {
  return index < sensorDriverCount() ? registeredDrivers[index] : nullptr;
}
