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

  // QMI8658 FIFO (datasheet section 8): both sensors at 224 Hz, since the FIFO
  // takes two sensors only at one rate, streamed into its 128 frames (571 ms),
  // the oldest dropped when it is full. A frame holds raw counts, acceleration
  // at QMI8658_COUNTS_PER_G and rotation at QMI8658_COUNTS_PER_DPS.
  struct RawFrame {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
  };
  static constexpr int32_t QMI8658_COUNTS_PER_G = 16384;   // ±2 g
  static constexpr int32_t QMI8658_COUNTS_PER_DPS = 64;    // ±512 dps
  static constexpr uint8_t FIFO_CHUNK = 8;                 // Frames per I2C read

  // Turns the FIFO on as above and leaves both sensors sampling. Returns false
  // when the IMU is absent or not a QMI8658, on I2C error or when the chip does
  // not finish a command; the begin() setup is then back.
  bool enableFifo();

  // Turns the FIFO off and puts back the begin() setup, both sensors sampling.
  // Returns false when absent or on I2C error.
  bool disableFifo();

  // Reads every frame the FIFO holds, oldest first, FIFO_CHUNK at a time, and
  // hands each chunk to `sink`; `gapBefore` is set on the first chunk when the
  // FIFO filled up and dropped frames since the last read. `frames` counts the
  // frames handed. Returns false when absent, on I2C error, or when the count
  // is not whole frames (the FIFO is then emptied); either way the FIFO takes
  // samples again.
  using FifoSink = void (*)(const RawFrame* frames, uint8_t count, bool gapBefore, void* context);
  bool readFifo(FifoSink sink, void* context, uint16_t& frames);

 private:
  bool begun_ = false;
  // The QMI8658 can legally appear at 0x6A or 0x6B depending on its SA0
  // strap. Keep the address found by begin() instead of repeatedly using the
  // board profile's preferred address.
  uint8_t addr_ = 0;
};

}  // namespace freeink

using Imu = freeink::Imu;
