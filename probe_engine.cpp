#include "probe_engine.h"

#include <Arduino.h>

#include "config.h"

// Safety scope: high impedance only means the firmware is not driving the
// probes. It is NOT protection against shorts, overvoltage, ESD or charged
// capacitors; that needs a designed hardware front end.

namespace {

enum class ProbeState : uint8_t {
  Idle,   // Inactive: no excitation, no measurement, no detection.
  Fault,  // Safe hold: probes restored to high impedance, stays inactive.
  // Future detection/measurement states (e.g. Detecting, Measuring,
  // Reporting) go here. Each must return the probes to resetTestPins() on
  // exit, and none should be added before the protection front end exists.
};

ProbeState state = ProbeState::Idle;

void enterFault() {
  resetTestPins();
  state = ProbeState::Fault;
}

}  // namespace

void resetTestPins() {
  // Plain INPUT: output driver off, internal pull-up and pull-down off.
  // Never pass through OUTPUT, INPUT_PULLUP or INPUT_PULLDOWN here.
  pinMode(PROBE_PIN_A, INPUT);
  pinMode(PROBE_PIN_B, INPUT);
  pinMode(PROBE_PIN_C, INPUT);
}

void initProbeEngine() {
  resetTestPins();
  state = ProbeState::Idle;
}

void probeTick() {
  switch (state) {
    case ProbeState::Idle:
      // v0.1: stay inactive. Floating digital reads mean nothing, so no
      // insertion detection here. A future phase would leave Idle for a
      // detection state on an explicit trigger.
      break;

    case ProbeState::Fault:
      // Probes were restored on entry; hold here until a future reset path.
      break;

    default:
      // Corrupt/unknown state: take the same safe path as a fault.
      enterFault();
      break;
  }
}
