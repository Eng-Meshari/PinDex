// Probe engine for the A/B/C test nodes. In v0.1 it is deliberately
// inactive: probes stay high-impedance inputs and nothing is measured.
#pragma once

// Resets the probes to high impedance and puts the engine in Idle.
void initProbeEngine();

// Sets every probe pin to plain INPUT (no pull-up/pull-down). Idempotent.
void resetTestPins();

// Advances the state machine; call on every loop() iteration.
void probeTick();
