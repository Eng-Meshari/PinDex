// Shipped backend: this board has NO protected four-wire interface. It
// reports no capabilities and refuses every connection, so the engine never
// schedules a FourWirePowered attempt (the driver is reported as "hardware
// required"). Replace this file only with an implementation for real,
// reviewed hardware (see README), and never report a capability the circuit
// does not have.

#include "four_wire_backend.h"

uint16_t fourWireBackendCapabilities() { return 0; }

bool fourWireBackendConnect(uint16_t, uint8_t, uint8_t, uint8_t, uint8_t) { return false; }

void fourWireBackendDisconnect() {}  // nothing exists to switch off
