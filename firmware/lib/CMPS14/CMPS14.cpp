#include "CMPS14.h"

bool CMPS14::begin(uint32_t i2cClockHz) {
  clockHz_ = i2cClockHz;
  wire_.begin();
  wire_.setClock(clockHz_);

  uint8_t v = 0;
  if (!readRegisters(kRegVersion, &v, 1) || v == 0x00 || v == 0xFF) return false;
  fwVersion_ = v;
  return true;
}

bool CMPS14::update() {
  uint8_t raw[2];
  if (readRegisters(kRegBearing16, raw, 2)) {
    // High byte first: bearing = (hi << 8) | lo, in tenths of a degree.
    const uint16_t tenths = (static_cast<uint16_t>(raw[0]) << 8) | raw[1];
    if (tenths <= 3599) {                       // reject corrupted frames
      headingDeg_ = tenths / 10.0;
      hasSample_ = true;
      consecutiveFailures_ = 0;
      return true;
    }
  }
  ++failures_;
  if (++consecutiveFailures_ >= kRecoverAfter) {
    recoverBus();
    consecutiveFailures_ = 0;
  }
  return false;
}

bool CMPS14::readCalibration(Calibration& out) {
  uint8_t c = 0;
  if (!readRegisters(kRegCalibState, &c, 1)) return false;
  out.system = (c >> 6) & 0x03;
  out.gyro   = (c >> 4) & 0x03;
  out.accel  = (c >> 2) & 0x03;
  out.mag    =  c       & 0x03;
  return true;
}

bool CMPS14::readRegisters(uint8_t reg, uint8_t* buf, uint8_t len) {
  wire_.beginTransmission(addr_);
  wire_.write(reg);
  if (wire_.endTransmission(false) != 0) return false;   // repeated start

  const uint8_t got = wire_.requestFrom(addr_, len);
  if (got != len) {
    while (wire_.available()) wire_.read();              // drain
    return false;
  }
  for (uint8_t i = 0; i < len; ++i) buf[i] = wire_.read();
  return true;
}

void CMPS14::recoverBus() {
  // Re-initialising the peripheral clears a wedged master state machine
  wire_.end();
  wire_.begin();
  wire_.setClock(clockHz_);
}