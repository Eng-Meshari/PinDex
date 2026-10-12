// Protected four-wire power/routing backend for FourWirePowered attempts.
// Link seam: the ESP32 build links four_wire_backend.cpp; host tests link a
// fake instead. Only the probe engine calls it.
//
// Nodes are socket indices (0 = A ... 3 = D), never GPIO numbers: a real
// routing circuit has its own control pins, which this firmware does not
// define. The probe GPIOs stay plain INPUT while the backend powers a
// mapping; the driver then signals on the SIG1/SIG2 GPIOs.
#pragma once

#include <stdint.h>

// kBackend* bits (sensor_driver.h) that the hardware actually provides.
uint16_t fourWireBackendCapabilities();

// Routes supply to vccNode and ground to gndNode, with ESP32-safe signal
// paths on sig1Node and sig2Node. The engine calls it only after
// fourWireBackendDisconnect() and with every probe GPIO plain INPUT.
// false = refused, nothing energised.
bool fourWireBackendConnect(uint8_t vccNode, uint8_t sig1Node, uint8_t sig2Node, uint8_t gndNode);

// Supply off and every route open. Idempotent and safe at any time.
void fourWireBackendDisconnect();
