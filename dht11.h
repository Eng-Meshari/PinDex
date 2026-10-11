// DHT11-compatible frame decoding. Pure functions: no GPIO, no timing, so
// they can be tested on a host. GPIO capture lives in probe_engine.cpp.
// DHT sensors use their own single-wire protocol; it is not Dallas/Maxim
// 1-Wire and needs no OneWire library.
#pragma once

#include <stdint.h>

#define DHT_FRAME_BITS 40
#define DHT_FRAME_BYTES 5

enum class DhtError : uint8_t {
  None,
  StuckLow,             // line never rose after release
  NoResponse,           // line stayed high: absent sensor, wrong mapping, stuck high
  AckTimeout,           // acknowledgement LOW/HIGH phase timed out
  BitTimeout,           // truncated frame, a bit phase or the frame deadline expired
  BadAckTiming,         // acknowledgement widths implausible
  BadBitTiming,         // a bit LOW/HIGH width implausible
  Checksum,             // byte 4 != (byte 0..3 sum) mod 256
  UnsupportedEncoding,  // not the supported DHT11 field layout
  OutOfRange,           // decoded value outside the DHT11 profile
};

// Measured phase widths of one sensor response, in microseconds.
struct DhtPulses {
  uint16_t ackLowUs;
  uint16_t ackHighUs;
  uint16_t bitLowUs[DHT_FRAME_BITS];
  uint16_t bitHighUs[DHT_FRAME_BITS];
};

struct DhtReading {
  int16_t temperatureDeciC;  // 0.1 degC units
  uint8_t humidityPct;
};

// Validates acknowledgement and bit widths, then packs 40 bits MSB first.
// bytes is zeroed first and stays zeroed on any error.
DhtError dhtPulsesToBytes(const DhtPulses &pulses, uint8_t bytes[DHT_FRAME_BYTES]);

// Modulo-256 checksum only; says nothing about whether values are plausible.
bool dhtChecksumOk(const uint8_t bytes[DHT_FRAME_BYTES]);

// Checksum, then the supported DHT11 encoding, then the plausibility window.
// Supported encoding: byte0 = RH integer, byte1 = 0, byte2 = T integer,
// byte3 = T tenths 0..9 (sign bit clear). Anything else is rejected.
DhtError dhtDecodeFrame(const uint8_t bytes[DHT_FRAME_BYTES], DhtReading &out);

const char *dhtErrorName(DhtError error);
