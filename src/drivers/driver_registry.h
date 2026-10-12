// Read-only view of the statically registered sensor drivers. Registry
// order is probe priority within each topology (see driver_registry.cpp).
#pragma once

#include <stddef.h>

#include "../../sensor_driver.h"

size_t sensorDriverCount();

// nullptr if index >= sensorDriverCount().
SensorDriver *sensorDriverAt(size_t index);
