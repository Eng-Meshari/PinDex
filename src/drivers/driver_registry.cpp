// The only place that knows concrete drivers. To add one: implement a
// SensorDriver in src/drivers/, include it here, define one static instance
// and add it to the table (earlier entries are tried first).
#include "driver_registry.h"

#include "dht11_driver.h"

namespace {

Dht11Driver dht11Driver;  // lives for the firmware's lifetime; never allocated

SensorDriver *const registeredDrivers[] = {
    &dht11Driver,
};

}  // namespace

size_t sensorDriverCount() { return sizeof registeredDrivers / sizeof registeredDrivers[0]; }

SensorDriver *sensorDriverAt(size_t index) {
  return index < sensorDriverCount() ? registeredDrivers[index] : nullptr;
}
