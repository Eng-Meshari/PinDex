// The only place that knows concrete drivers. To add one: implement a
// SensorDriver in src/drivers/, include it here, define one static instance
// and add it to the table. Priority: the engine tries the active protocols
// first (three-wire, then four-wire), then two-terminal (passive)
// interpretation; earlier entries go first within a topology. This table
// keeps that order: DHT11, ultrasonic, passive.
#include "driver_registry.h"

#include "dht11_driver.h"
#include "passive_driver.h"
#include "ultrasonic_driver.h"

namespace {

// Live for the firmware's lifetime; never allocated.
Dht11Driver dht11Driver;
UltrasonicDriver ultrasonicDriver;
PassiveDriver passiveDriver;

SensorDriver *const registeredDrivers[] = {
    &dht11Driver,
    &ultrasonicDriver,
    &passiveDriver,
};

}  // namespace

size_t sensorDriverCount() { return sizeof registeredDrivers / sizeof registeredDrivers[0]; }

SensorDriver *sensorDriverAt(size_t index) {
  return index < sensorDriverCount() ? registeredDrivers[index] : nullptr;
}
