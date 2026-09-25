#include <Arduino.h>
#include <Wire.h>

class CMPS14 {
 public:
  static constexpr uint8_t kDefaultAddress = 0x60;

  struct Calibration {
    uint8_t system = 0, gyro = 0, accel = 0, mag = 0;   // each 0 (bad) .. 3 (good)
    bool fullyCalibrated() const {
      return system == 3 && gyro == 3 && accel == 3 && mag == 3;
    }
  };

  explicit CMPS14(TwoWire& wire = Wire, uint8_t address = kDefaultAddress)
      : wire_(wire), addr_(address) {}

  /** Starts I2C and verifies the sensor answers. @return true on success. */
  bool begin(uint32_t i2cClockHz = 400000);

  /** Polls a new bearing. @return true if a valid sample was obtained. */
  bool update();

  /** Last valid bearing, degrees in [0, 360), clockwise-positive. */
  double   headingDeg()      const { return headingDeg_; }
  bool     hasValidSample()  const { return hasSample_; }
  uint8_t  firmwareVersion() const { return fwVersion_; }
  uint32_t failureCount()    const { return failures_; }

  /** Reads the calibration status register (0x1E). */
  bool readCalibration(Calibration& out);

 private:
  static constexpr uint8_t kRegVersion    = 0x00;
  static constexpr uint8_t kRegBearing16  = 0x02;
  static constexpr uint8_t kRegCalibState = 0x1E;
  static constexpr uint8_t kRecoverAfter  = 5;   // consecutive failures

  bool readRegisters(uint8_t reg, uint8_t* buf, uint8_t len);
  void recoverBus();

  TwoWire& wire_;
  uint8_t  addr_;
  uint32_t clockHz_ = 400000;
  double   headingDeg_ = 0.0;
  bool     hasSample_ = false;
  uint8_t  fwVersion_ = 0;
  uint8_t  consecutiveFailures_ = 0;
  uint32_t failures_ = 0;  
};

