#include "dht11.h"

#include <string.h>

#include "config.h"

namespace {

bool within(uint16_t value, uint16_t lo, uint16_t hi) {
  return value >= lo && value <= hi;
}

}  // namespace

DhtError dhtPulsesToBytes(const DhtPulses &pulses, uint8_t bytes[DHT_FRAME_BYTES]) {
  memset(bytes, 0, DHT_FRAME_BYTES);

  if (!within(pulses.ackLowUs, DHT_ACK_MIN_US, DHT_ACK_MAX_US) ||
      !within(pulses.ackHighUs, DHT_ACK_MIN_US, DHT_ACK_MAX_US)) {
    return DhtError::BadAckTiming;
  }

  for (uint8_t i = 0; i < DHT_FRAME_BITS; i++) {
    if (!within(pulses.bitLowUs[i], DHT_BIT_LOW_MIN_US, DHT_BIT_LOW_MAX_US) ||
        !within(pulses.bitHighUs[i], DHT_BIT_HIGH_MIN_US, DHT_BIT_HIGH_MAX_US)) {
      memset(bytes, 0, DHT_FRAME_BYTES);  // never hand out partial data
      return DhtError::BadBitTiming;
    }
    const uint8_t bit = pulses.bitHighUs[i] > DHT_BIT_THRESHOLD_US ? 1 : 0;
    bytes[i / 8] = static_cast<uint8_t>((bytes[i / 8] << 1) | bit);
  }
  return DhtError::None;
}

bool dhtChecksumOk(const uint8_t bytes[DHT_FRAME_BYTES]) {
  const uint8_t sum = static_cast<uint8_t>(bytes[0] + bytes[1] + bytes[2] + bytes[3]);
  return sum == bytes[4];
}

DhtError dhtDecodeFrame(const uint8_t bytes[DHT_FRAME_BYTES], DhtReading &out) {
  if (!dhtChecksumOk(bytes)) return DhtError::Checksum;

  // Non-zero humidity decimal (e.g. DHT22/AM2302 16-bit fields) or a
  // temperature byte3 above 9 (incl. the 0x80 negative-sign bit some newer
  // DHT11 parts use) is a different encoding: reject, don't reinterpret.
  if (bytes[1] != 0 || bytes[3] > 9) return DhtError::UnsupportedEncoding;

  const int humidity = bytes[0];
  const int temperature = bytes[2];
  if (humidity < DHT11_HUMIDITY_MIN_PCT || humidity > DHT11_HUMIDITY_MAX_PCT ||
      temperature < DHT11_TEMPERATURE_MIN_C || temperature > DHT11_TEMPERATURE_MAX_C) {
    return DhtError::OutOfRange;
  }

  out.humidityPct = bytes[0];
  out.temperatureDeciC = static_cast<int16_t>(temperature * 10 + bytes[3]);
  return DhtError::None;
}

const char *dhtErrorName(DhtError error) {
  switch (error) {
    case DhtError::None: return "none";
    case DhtError::StuckLow: return "stuck-low";
    case DhtError::NoResponse: return "no-response";
    case DhtError::AckTimeout: return "ack-timeout";
    case DhtError::BitTimeout: return "bit-timeout";
    case DhtError::BadAckTiming: return "bad-ack-timing";
    case DhtError::BadBitTiming: return "bad-bit-timing";
    case DhtError::Checksum: return "checksum";
    case DhtError::UnsupportedEncoding: return "unsupported-encoding";
    case DhtError::OutOfRange: return "out-of-range";
  }
  return "unknown";
}
