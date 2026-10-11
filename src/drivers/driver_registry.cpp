// The only place that knows concrete drivers. To add one: implement a
// SensorDriver in src/drivers/, include it here, define one static instance
// and add it to the table. Earlier entries are tried first among drivers of
// the same topology; the engine always tries powered (three-wire)
// protocols before two-terminal (passive) interpretation.
#include "driver_registry.h"

#include "dht11_driver.h"
#include "passive_driver.h"

namespace {

// Live for the firmware's lifetime; never allocated.
Dht11Driver dht11Driver;
PassiveDriver passiveDriver;

SensorDriver *const registeredDrivers[] = {
    &dht11Driver,
    &passiveDriver,
};

}  // namespace

size_t sensorDriverCount() { return sizeof registeredDrivers / sizeof registeredDrivers[0]; }

SensorDriver *sensorDriverAt(size_t index) {
  return index < sensorDriverCount() ? registeredDrivers[index] : nullptr;
}
