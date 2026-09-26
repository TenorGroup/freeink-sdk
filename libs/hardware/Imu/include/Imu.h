#pragma once

// FreeInk inertial measurement unit (LSM6DS3TR-C or QMI8658, 6-axis accel +
// gyro).
//
// Reads acceleration (g) and angular rate (deg/s) from the I2C IMU described by
// BoardConfig::ACTIVE.sensors (imuAddr / sensor bus). Dependency-free Wire
// access, mirroring BatteryMonitor. Boards without an IMU (FREEINK_CAP_IMU off,
// or imuAddr == 0) link stub bodies and present() returns false.

#include <Arduino.h>

#include <cstdint>

namespace freeink {

class Imu {
 public:
  struct Sample {
    float ax, ay, az;  // acceleration, g (1 g ~= 9.81 m/s^2)
    float gx, gy, gz;  // angular rate, degrees/second
  };

  // Verifies WHO_AM_I and configures accel + gyro for the active board.
  // Returns false when the active board has no IMU or the part doesn't identify.
  bool begin();
  bool present() const { return begun_; }

  // Reads one accel + gyro sample. Returns false on I2C error.
  bool read(Sample& out);

  // Puts the sensors into hardware standby / power-down. Config registers are
  // retained, so wake() restores sampling without a full begin(). Returns
  // false when the IMU is absent or on I2C error.
  bool sleep();

  // Restarts sampling after sleep(). Returns false when absent or on I2C
  // error; allow for a settling transient before trusting samples.
  bool wake();

  // QMI8658 tap engine (datasheet section 10). Windows count accelerometer
  // samples, alpha and gamma are in 1/128, thresholds in 0.001 g^2.
  struct TapConfig {
    uint8_t priority;
    uint8_t peakWindow;
    uint16_t tapWindow;
    uint16_t doubleTapWindow;
    uint8_t alpha;
    uint8_t gamma;
    uint16_t peakThreshold;
    uint16_t quietThreshold;
  };

  // Loads `config` into the tap engine, turns it on with the accelerometer at
  // 224 Hz (the gyro keeps begin()'s rate) and leaves both sensors sampling.
  // Returns false when the IMU is absent or not a QMI8658, on I2C error or
  // when the chip does not finish a command; the begin() setup is then back.
  bool enableTap(const TapConfig& config);

  // Turns the tap engine off and puts back the begin() setup, both sensors
  // sampling. Returns false when absent or on I2C error.
  bool disableTap();

  // `taps` is 0 when the chip reports no tap, else its count (1 single,
  // 2 double). Returns false when absent or on I2C error.
  bool readTap(uint8_t& taps);

 private:
  bool begun_ = false;
  // The QMI8658 can legally appear at 0x6A or 0x6B depending on its SA0
  // strap. Keep the address found by begin() instead of repeatedly using the
  // board profile's preferred address.
  uint8_t addr_ = 0;
};

}  // namespace freeink

using Imu = freeink::Imu;
