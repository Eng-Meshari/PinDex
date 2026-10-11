// Host-side stand-in for the parts of the ESP32 Arduino core PinDex uses.
// Values mirror esp32-hal-gpio.h (core 3.x). Time is a 64-bit microsecond
// counter; millis()/micros() truncate to 32 bits exactly like the ESP32
// (both derive from esp_timer), so rollover behaves the same.
#pragma once

#include <stddef.h>
#include <stdint.h>

#define LOW 0x0
#define HIGH 0x1
#define INPUT 0x01
#define OUTPUT 0x03
#define PULLUP 0x04
#define INPUT_PULLUP 0x05
#define PULLDOWN 0x08
#define INPUT_PULLDOWN 0x09
#define OPEN_DRAIN 0x10
#define OUTPUT_OPEN_DRAIN 0x13
#define ANALOG 0xC0  // used by the simulated ADC adapter to mark a sensed pad

namespace mock {

struct Pin {
  uint8_t mode;  // 0 = never configured
  uint8_t latch;
};

struct Event {
  uint64_t us;
  uint8_t pin;
  uint8_t mode;   // pin state after the call
  uint8_t latch;
  bool isPinMode;
};

constexpr size_t kMaxEvents = 50000;

inline Pin pins[64];
inline uint64_t nowUs = 0;
inline uint32_t microsOffset = 0;  // lets a test wrap micros() at a chosen instant
inline Event events[kMaxEvents];
inline size_t eventCount = 0;
inline bool eventOverflow = false;
inline int criticalDepth = 0;
inline uint64_t criticalSinceUs = 0;
inline uint64_t criticalMaxUs = 0;
inline int (*readHook)(uint8_t pin) = nullptr;  // -1 = use the pin's own level
inline void (*changeHook)(uint8_t pin, bool isPinMode) = nullptr;

inline bool isOutput(uint8_t pin) { return (pins[pin].mode & OUTPUT) == OUTPUT; }
inline bool drivesHigh(uint8_t pin) {
  return isOutput(pin) && !(pins[pin].mode & OPEN_DRAIN) && pins[pin].latch == HIGH;
}
inline bool drivesLow(uint8_t pin) { return isOutput(pin) && pins[pin].latch == LOW; }

inline int ownLevel(uint8_t pin) {
  if (drivesHigh(pin)) return HIGH;
  if (drivesLow(pin)) return LOW;
  return (pins[pin].mode & PULLUP) ? HIGH : LOW;  // a floating input reads LOW here
}

inline void record(uint8_t pin, bool isPinMode) {
  if (eventCount < kMaxEvents) {
    events[eventCount++] = Event{nowUs, pin, pins[pin].mode, pins[pin].latch, isPinMode};
  } else {
    eventOverflow = true;
  }
  if (changeHook) changeHook(pin, isPinMode);
}

inline void reset(uint64_t startUs) {
  for (Pin &p : pins) p = Pin{0, LOW};
  nowUs = startUs;
  microsOffset = 0;
  eventCount = 0;
  eventOverflow = false;
  criticalDepth = 0;
  criticalMaxUs = 0;
  readHook = nullptr;
  changeHook = nullptr;
}

}  // namespace mock

inline void pinMode(uint8_t pin, uint8_t mode) {
  mock::pins[pin].mode = mode;
  mock::record(pin, true);
}

inline void digitalWrite(uint8_t pin, uint8_t val) {
  mock::pins[pin].latch = val ? HIGH : LOW;
  mock::record(pin, false);
}

// Every read and every micros() call costs 1 us, so bounded polling loops
// always make progress in simulated time.
inline int digitalRead(uint8_t pin) {
  mock::nowUs += 1;
  if (mock::readHook) {
    const int v = mock::readHook(pin);
    if (v >= 0) return v;
  }
  return mock::ownLevel(pin);
}

inline unsigned long micros() {
  mock::nowUs += 1;
  return static_cast<uint32_t>(static_cast<uint32_t>(mock::nowUs) + mock::microsOffset);
}

inline unsigned long millis() { return static_cast<uint32_t>(mock::nowUs / 1000); }

// FreeRTOS critical-section stand-ins: track nesting and the longest hold.
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0

inline void portENTER_CRITICAL(portMUX_TYPE *) {
  if (mock::criticalDepth++ == 0) mock::criticalSinceUs = mock::nowUs;
}

inline void portEXIT_CRITICAL(portMUX_TYPE *) {
  if (--mock::criticalDepth == 0) {
    const uint64_t held = mock::nowUs - mock::criticalSinceUs;
    if (held > mock::criticalMaxUs) mock::criticalMaxUs = held;
  }
}
