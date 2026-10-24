#include "Imu.h"

#include <BoardConfig.h>

#if FREEINK_CAP_IMU

#include <Wire.h>
#include <soc/soc_caps.h>

namespace freeink {
namespace {

// LSM6DS3TR-C register map (datasheet).
constexpr uint8_t REG_WHO_AM_I = 0x0F;
constexpr uint8_t WHO_AM_I_VALUE = 0x6A;  // shared by LSM6DS3 / LSM6DS3TR-C
constexpr uint8_t REG_CTRL1_XL = 0x10;    // accel: ODR + full scale
constexpr uint8_t REG_CTRL2_G = 0x11;     // gyro: ODR + full scale
constexpr uint8_t REG_CTRL3_C = 0x12;     // BDU / auto-increment
constexpr uint8_t REG_OUTX_L_G = 0x22;    // gyro X..Z (6 bytes, LE)
constexpr uint8_t REG_OUTX_L_XL = 0x28;   // accel X..Z (6 bytes, LE)

// 104 Hz (ODR = 0100b in bits [7:4]); accel FS = ±2 g, gyro FS = ±245 dps (00b).
constexpr uint8_t CTRL1_XL_104HZ_2G = 0x40;
constexpr uint8_t CTRL2_G_104HZ_245DPS = 0x40;
constexpr uint8_t CTRL3_C_BDU_IF_INC = 0x44;  // BDU=1, IF_INC=1 (block update + auto-increment)

// Sensitivities for the scales above (datasheet "mechanical characteristics").
constexpr float ACCEL_G_PER_LSB = 0.061f / 1000.0f;  // 0.061 mg/LSB at ±2 g
constexpr float GYRO_DPS_PER_LSB = 8.75f / 1000.0f;  // 8.75 mdps/LSB at ±245 dps

// QMI8658 register map.
constexpr uint8_t QMI8658_REG_WHO_AM_I = 0x00;
constexpr uint8_t QMI8658_WHO_AM_I_VALUE = 0x05;
constexpr uint8_t QMI8658_REG_CTRL1 = 0x02;
constexpr uint8_t QMI8658_REG_CTRL2 = 0x03;
constexpr uint8_t QMI8658_REG_CTRL3 = 0x04;
constexpr uint8_t QMI8658_REG_CTRL7 = 0x08;
constexpr uint8_t QMI8658_REG_CTRL8 = 0x09;
constexpr uint8_t QMI8658_REG_CTRL9 = 0x0A;
constexpr uint8_t QMI8658_REG_FIFO_CTRL = 0x14;
constexpr uint8_t QMI8658_REG_FIFO_SMPL_CNT = 0x15;  // FIFO_STATUS follows at 0x16
constexpr uint8_t QMI8658_REG_FIFO_DATA = 0x17;
constexpr uint8_t QMI8658_REG_STATUSINT = 0x2D;
constexpr uint8_t QMI8658_REG_AX_L = 0x35;
constexpr uint8_t QMI8658_REG_GX_L = 0x3B;
constexpr uint8_t QMI8658_ADDR_6A = 0x6A;
constexpr uint8_t QMI8658_ADDR_6B = 0x6B;
constexpr uint8_t QMI8658_CTRL1_BIG_ENDIAN = 1U << 5;
constexpr uint8_t QMI8658_CTRL1_AUTO_INC = 1U << 6;
constexpr uint8_t QMI8658_CTRL1_SENSOR_DISABLE = 1U << 0;
constexpr uint8_t QMI8658_CTRL1_BASE = QMI8658_CTRL1_AUTO_INC | QMI8658_CTRL1_BIG_ENDIAN;
constexpr uint8_t QMI8658_CTRL2_FS_2G = 0U << 4;
constexpr uint8_t QMI8658_CTRL2_ODR_28HZ = 0x08;
constexpr uint8_t QMI8658_CTRL3_FS_512DPS = 0b101U << 4;
constexpr uint8_t QMI8658_CTRL3_ODR_28HZ = 0x08;
constexpr uint8_t QMI8658_CTRL7_ACC_GYRO_ENABLE = 0x03;
constexpr uint8_t QMI8658_CTRL7_DISABLE_ALL = 0x00;
// FIFO: both sensors at ODR code 5 (224.2 Hz), the FIFO in stream mode
// (oldest frame dropped when full) 128 frames deep, CTRL9 done flag read from
// STATUSINT bit 7 (CTRL8 bit 7) since no interrupt line is wired.
constexpr uint8_t QMI8658_CTRL2_ODR_224HZ = 0x05;
constexpr uint8_t QMI8658_CTRL3_ODR_224HZ = 0x05;
constexpr uint8_t QMI8658_CTRL8_HANDSHAKE_STATUSINT = 1U << 7;
constexpr uint8_t QMI8658_CTRL8_DEFAULT = 0x00;
constexpr uint8_t QMI8658_CTRL9_ACK = 0x00;
constexpr uint8_t QMI8658_CTRL9_RST_FIFO = 0x04;
constexpr uint8_t QMI8658_CTRL9_REQ_FIFO = 0x05;
constexpr uint8_t QMI8658_FIFO_CTRL_STREAM_128 = (0b11U << 2) | 0b10U;
constexpr uint8_t QMI8658_FIFO_CTRL_BYPASS = 0x00;
constexpr uint8_t QMI8658_FIFO_STATUS_FULL = 1U << 7;
constexpr uint8_t QMI8658_FIFO_STATUS_OVERFLOW = 1U << 5;
constexpr uint8_t QMI8658_FIFO_FRAME_BYTES = 12;  // accelerometer then gyro, 16 bits each, low byte first
constexpr uint8_t QMI8658_STATUSINT_CMD_DONE = 1U << 7;
constexpr unsigned long QMI8658_CTRL9_TIMEOUT_MS = 20;
// LSM6DS3: ODR bits [7:4] = 0000b powers the sensor down; full-scale bits are
// retained, so restoring the configured CTRL value resumes sampling.
constexpr uint8_t CTRL_ODR_POWER_DOWN = 0x00;
constexpr float QMI8658_ACCEL_G_PER_LSB = 1.0f / 16384.0f;  // ±2 g
constexpr float QMI8658_GYRO_DPS_PER_LSB = 1.0f / 64.0f;    // ±512 dps

// SC7A20H datasheet v1.1, sections 11.1 / 13. Unlike LIS3DH, HR is
// CTRL0 bit 0 (0x1F), not CTRL4 bit 3 (which is a low-pass filter bit).
constexpr uint8_t SC7_ID = 0x11;
constexpr uint8_t SC7_VERSION_REG = 0x70;
constexpr uint8_t SC7_VERSION = 0x28;
constexpr uint8_t SC7_CTRL0 = 0x1F;
constexpr uint8_t SC7_CTRL1 = 0x20;
constexpr uint8_t SC7_CTRL4 = 0x23;
constexpr uint8_t SC7_STATUS = 0x27;
constexpr uint8_t SC7_OUT = 0x28 | 0x80;  // I2C auto-increment burst
constexpr uint8_t SC7_POWER_DOWN = 0x07;
constexpr uint8_t SC7_100HZ = 0x57;  // XYZ on, LPen off
constexpr uint8_t SC7_DIG_CTRL = 0x57;
constexpr float SC7_G_PER_RAW_LSB = 1.0f / 16384.0f;  // section 13.13: 0x4000 = 1 g at ±2 g

bool g_wireReady[2] = {false, false};
TwoWire& sensorWire() {
#if SOC_I2C_NUM > 1
  const auto& s = BoardConfig::ACTIVE.sensors;
  return s.i2cBus == 1 ? Wire1 : Wire;
#else
  return Wire;
#endif
}

void ensureWire() {
  const auto& s = BoardConfig::ACTIVE.sensors;
  const uint8_t bus =
#if SOC_I2C_NUM > 1
      s.i2cBus == 1 ? 1 : 0;
#else
      0;
#endif
  if (g_wireReady[bus]) return;
  if (s.i2cSda < 0 || s.i2cScl < 0) return;  // no sensor bus on this board
  auto& wire = sensorWire();
  wire.begin(s.i2cSda, s.i2cScl, s.i2cHz);
  g_wireReady[bus] = true;
}

bool writeReg(uint8_t addr, uint8_t reg, uint8_t value) {
  ensureWire();
  auto& wire = sensorWire();
  wire.beginTransmission(addr);
  wire.write(reg);
  wire.write(value);
  return wire.endTransmission() == 0;
}

bool readRegs(uint8_t addr, uint8_t reg, uint8_t* dst, uint8_t len) {
  ensureWire();
  auto& wire = sensorWire();
  wire.beginTransmission(addr);
  wire.write(reg);
  if (wire.endTransmission(false) != 0) return false;
  if (wire.requestFrom(addr, len, static_cast<uint8_t>(true)) < len) return false;
  for (uint8_t i = 0; i < len; ++i) dst[i] = wire.read();
  return true;
}

bool qmi8658PresentAt(uint8_t addr) {
  uint8_t who = 0;
  return readRegs(addr, QMI8658_REG_WHO_AM_I, &who, 1) && who == QMI8658_WHO_AM_I_VALUE;
}

bool sc7a20hPresentAt(uint8_t addr) {
  uint8_t who = 0, version = 0;
  return readRegs(addr, REG_WHO_AM_I, &who, 1) && who == SC7_ID &&
         readRegs(addr, SC7_VERSION_REG, &version, 1) && version == SC7_VERSION;
}

bool configureSc7a20h(uint8_t addr) {
  // Disable sampling while configuring. No FIFO, high-pass filter or IRQ
  // routing: gravity remains available for tilt/orientation consumers.
  if (!writeReg(addr, SC7_CTRL1, SC7_POWER_DOWN)) return false;
  if (!writeReg(addr, SC7_CTRL0, 0x01) || !writeReg(addr, 0x21, 0x00) ||
      !writeReg(addr, 0x22, 0x00) || !writeReg(addr, SC7_CTRL4, 0x80) ||
      !writeReg(addr, 0x24, 0x00) || !writeReg(addr, 0x25, 0x00) ||
      !writeReg(addr, 0x2E, 0x00)) return false;
  if (addr == 0x18) {
    // SA0 low: disable its internal pull-up to avoid leakage, preserving the
    // separate SDA/SCL pull-up setting (section 13.30).
    uint8_t dig = 0;
    if (!readRegs(addr, SC7_DIG_CTRL, &dig, 1) || !writeReg(addr, SC7_DIG_CTRL, dig | 0x08)) return false;
  }
  return writeReg(addr, SC7_CTRL1, SC7_100HZ);
}

bool powerDownQmi8658(uint8_t addr) {
  // Do not short-circuit these writes: even if disabling the sensor engines
  // fails, still try to stop the internal oscillator. This is also used as the
  // cleanup path after a partially failed begin().
  const bool sensorsDisabled = writeReg(addr, QMI8658_REG_CTRL7, QMI8658_CTRL7_DISABLE_ALL);
  const bool oscillatorDisabled = writeReg(addr, QMI8658_REG_CTRL1, QMI8658_CTRL1_BASE | QMI8658_CTRL1_SENSOR_DISABLE);
  return sensorsDisabled && oscillatorDisabled;
}

// Waits for STATUSINT's CTRL9 done flag to read `done`, polling each millisecond.
bool qmi8658WaitCmdDone(uint8_t addr, bool done) {
  const unsigned long start = millis();
  for (;;) {
    uint8_t status = 0;
    if (!readRegs(addr, QMI8658_REG_STATUSINT, &status, 1)) return false;
    if (((status & QMI8658_STATUSINT_CMD_DONE) != 0) == done) return true;
    if (millis() - start >= QMI8658_CTRL9_TIMEOUT_MS) return false;
    delay(1);
  }
}

// The CTRL9 protocol: command, wait for done, acknowledge, wait for the flag to drop.
bool qmi8658Ctrl9(uint8_t addr, uint8_t command) {
  return writeReg(addr, QMI8658_REG_CTRL9, command) && qmi8658WaitCmdDone(addr, true) &&
         writeReg(addr, QMI8658_REG_CTRL9, QMI8658_CTRL9_ACK) && qmi8658WaitCmdDone(addr, false);
}

// The begin() setup with the FIFO off, both sensors sampling.
bool qmi8658FifoOff(uint8_t addr) {
  return writeReg(addr, QMI8658_REG_CTRL7, QMI8658_CTRL7_DISABLE_ALL) &&
         writeReg(addr, QMI8658_REG_FIFO_CTRL, QMI8658_FIFO_CTRL_BYPASS) &&
         writeReg(addr, QMI8658_REG_CTRL8, QMI8658_CTRL8_DEFAULT) &&
         writeReg(addr, QMI8658_REG_CTRL2, QMI8658_CTRL2_FS_2G | QMI8658_CTRL2_ODR_28HZ) &&
         writeReg(addr, QMI8658_REG_CTRL3, QMI8658_CTRL3_FS_512DPS | QMI8658_CTRL3_ODR_28HZ) &&
         writeReg(addr, QMI8658_REG_CTRL7, QMI8658_CTRL7_ACC_GYRO_ENABLE);
}

}  // namespace

bool Imu::begin() {
  begun_ = false;
  sleeping_ = false;
  addr_ = 0;

  const auto& s = BoardConfig::ACTIVE.sensors;
  const uint8_t configuredAddr = s.imuAddr;
  if (configuredAddr == 0) return false;
  if (s.i2cSda < 0 || s.i2cScl < 0 || s.i2cHz == 0) return false;
#if FREEINK_DEVICE_METALIO_EINK4
  if (BoardConfig::isMetalioEInk4() && !metalio::ensureBooted()) return false;
#endif
  ensureWire();
  uint8_t who = 0;
  switch (s.imuType) {
    case BoardConfig::ImuType::Lsm6ds3:
      if (!readRegs(configuredAddr, REG_WHO_AM_I, &who, 1) || who != WHO_AM_I_VALUE) return false;
      if (!writeReg(configuredAddr, REG_CTRL3_C, CTRL3_C_BDU_IF_INC)) return false;
      if (!writeReg(configuredAddr, REG_CTRL1_XL, CTRL1_XL_104HZ_2G)) return false;
      if (!writeReg(configuredAddr, REG_CTRL2_G, CTRL2_G_104HZ_245DPS)) return false;
      addr_ = configuredAddr;
      break;
    case BoardConfig::ImuType::Qmi8658: {
      // SA0 selects between 0x6A and 0x6B. X3 production revisions have used
      // both, so treat the profile address as a preference rather than a
      // guarantee. This restores the fallback used by the pre-SDK X3 driver.
      const uint8_t alternateAddr = configuredAddr == QMI8658_ADDR_6A ? QMI8658_ADDR_6B : QMI8658_ADDR_6A;
      if (qmi8658PresentAt(configuredAddr)) {
        addr_ = configuredAddr;
      } else if (qmi8658PresentAt(alternateAddr)) {
        addr_ = alternateAddr;
      } else {
        return false;
      }

      const bool configured = writeReg(addr_, QMI8658_REG_CTRL7, QMI8658_CTRL7_DISABLE_ALL) &&
                              writeReg(addr_, QMI8658_REG_CTRL1, QMI8658_CTRL1_BASE) &&
                              writeReg(addr_, QMI8658_REG_CTRL2, QMI8658_CTRL2_FS_2G | QMI8658_CTRL2_ODR_28HZ) &&
                              writeReg(addr_, QMI8658_REG_CTRL3, QMI8658_CTRL3_FS_512DPS | QMI8658_CTRL3_ODR_28HZ) &&
                              writeReg(addr_, QMI8658_REG_CTRL7, QMI8658_CTRL7_ACC_GYRO_ENABLE);
      if (!configured) {
        // A failed setup must not strand a previously running sensor in its
        // multi-milliamp active mode. The digital interface remains available
        // in QMI8658 power-down, so this cleanup is safe to attempt here.
        powerDownQmi8658(addr_);
        return false;
      }
      break;
    }
    case BoardConfig::ImuType::Sc7a20h: {
      if (configuredAddr != 0x18 && configuredAddr != 0x19) return false;
      const uint8_t alternate = configuredAddr == 0x19 ? 0x18 : 0x19;
      if (sc7a20hPresentAt(configuredAddr)) addr_ = configuredAddr;
      else if (sc7a20hPresentAt(alternate)) addr_ = alternate;
      else return false;
      if (!configureSc7a20h(addr_)) {
        writeReg(addr_, SC7_CTRL1, SC7_POWER_DOWN);
        addr_ = 0;
        return false;
      }
      delay(10);  // one 100 Hz sample period, also exceeds the 1 ms startup time
      break;
    }
    case BoardConfig::ImuType::None:
      return false;
  }
  begun_ = true;
  return true;
}

bool Imu::hasGyroscope() const {
  const auto type = BoardConfig::ACTIVE.sensors.imuType;
  return begun_ && (type == BoardConfig::ImuType::Lsm6ds3 || type == BoardConfig::ImuType::Qmi8658);
}

bool Imu::read(Sample& out) {
  const uint8_t addr = addr_;
  if (!begun_ || addr == 0) return false;
  const auto& s = BoardConfig::ACTIVE.sensors;
  if (s.imuType == BoardConfig::ImuType::Sc7a20h) {
    if (sleeping_) return false;
    uint8_t status = 0, raw[6] = {};
    if (!readRegs(addr, SC7_STATUS, &status, 1) || !(status & 0x08)) return false;
    if (!readRegs(addr, SC7_OUT, raw, sizeof(raw))) return false;
    // 12-bit two's complement, left-aligned; low four bits are not data.
    const int16_t ax = static_cast<int16_t>((raw[0] & 0xF0) | (uint16_t(raw[1]) << 8));
    const int16_t ay = static_cast<int16_t>((raw[2] & 0xF0) | (uint16_t(raw[3]) << 8));
    const int16_t az = static_cast<int16_t>((raw[4] & 0xF0) | (uint16_t(raw[5]) << 8));
    out = {ax * SC7_G_PER_RAW_LSB, ay * SC7_G_PER_RAW_LSB, az * SC7_G_PER_RAW_LSB, 0, 0, 0};
    return true;
  }
  uint8_t g[6] = {};
  uint8_t a[6] = {};
  if (s.imuType == BoardConfig::ImuType::Lsm6ds3) {
    if (!readRegs(addr, REG_OUTX_L_G, g, sizeof(g))) return false;
    if (!readRegs(addr, REG_OUTX_L_XL, a, sizeof(a))) return false;
  } else if (s.imuType == BoardConfig::ImuType::Qmi8658) {
    if (!readRegs(addr, QMI8658_REG_GX_L, g, sizeof(g))) return false;
    if (!readRegs(addr, QMI8658_REG_AX_L, a, sizeof(a))) return false;
  } else {
    return false;
  }

  const int16_t gx = static_cast<int16_t>(g[0] | g[1] << 8);
  const int16_t gy = static_cast<int16_t>(g[2] | g[3] << 8);
  const int16_t gz = static_cast<int16_t>(g[4] | g[5] << 8);
  const int16_t ax = static_cast<int16_t>(a[0] | a[1] << 8);
  const int16_t ay = static_cast<int16_t>(a[2] | a[3] << 8);
  const int16_t az = static_cast<int16_t>(a[4] | a[5] << 8);

  const float accelScale = s.imuType == BoardConfig::ImuType::Qmi8658 ? QMI8658_ACCEL_G_PER_LSB : ACCEL_G_PER_LSB;
  const float gyroScale = s.imuType == BoardConfig::ImuType::Qmi8658 ? QMI8658_GYRO_DPS_PER_LSB : GYRO_DPS_PER_LSB;
  out.ax = ax * accelScale;
  out.ay = ay * accelScale;
  out.az = az * accelScale;
  out.gx = gx * gyroScale;
  out.gy = gy * gyroScale;
  out.gz = gz * gyroScale;

  // Mount correction (see SensorsConfig): swap first, then flip, applied to
  // accel and gyro together so both report the same board frame.
  if (s.imuSwapXY) {
    float t = out.ax;
    out.ax = out.ay;
    out.ay = t;
    t = out.gx;
    out.gx = out.gy;
    out.gy = t;
  }
  if (s.imuFlipX) {
    out.ax = -out.ax;
    out.gx = -out.gx;
  }
  if (s.imuFlipY) {
    out.ay = -out.ay;
    out.gy = -out.gy;
  }
  return true;
}

bool Imu::sleep() {
  const uint8_t addr = addr_;
  if (!begun_ || addr == 0) return false;
  switch (BoardConfig::ACTIVE.sensors.imuType) {
    case BoardConfig::ImuType::Lsm6ds3:
      return writeReg(addr, REG_CTRL1_XL, CTRL_ODR_POWER_DOWN) && writeReg(addr, REG_CTRL2_G, CTRL_ODR_POWER_DOWN);
    case BoardConfig::ImuType::Qmi8658:
      // CTRL7 only disables sampling; the internal oscillator keeps running.
      // SensorDisable is required for the QMI8658's full power-down mode.
      return powerDownQmi8658(addr);
    case BoardConfig::ImuType::Sc7a20h:
      if (!writeReg(addr, SC7_CTRL1, SC7_POWER_DOWN)) return false;
      sleeping_ = true;
      return true;
    case BoardConfig::ImuType::None:
      return false;
  }
  return false;
}

bool Imu::wake() {
  const uint8_t addr = addr_;
  if (!begun_ || addr == 0) return false;
  switch (BoardConfig::ACTIVE.sensors.imuType) {
    case BoardConfig::ImuType::Lsm6ds3:
      return writeReg(addr, REG_CTRL1_XL, CTRL1_XL_104HZ_2G) && writeReg(addr, REG_CTRL2_G, CTRL2_G_104HZ_245DPS);
    case BoardConfig::ImuType::Qmi8658:
      // Re-enable the internal oscillator before restarting the sensors.
      return writeReg(addr, QMI8658_REG_CTRL1, QMI8658_CTRL1_BASE) &&
             writeReg(addr, QMI8658_REG_CTRL7, QMI8658_CTRL7_ACC_GYRO_ENABLE);
    case BoardConfig::ImuType::Sc7a20h:
      if (!writeReg(addr, SC7_CTRL1, SC7_100HZ)) return false;
      delay(10);
      sleeping_ = false;
      return true;
    case BoardConfig::ImuType::None:
      return false;
  }
  return false;
}

bool Imu::enableFifo() {
  const uint8_t addr = addr_;
  if (!begun_ || addr == 0 || BoardConfig::ACTIVE.sensors.imuType != BoardConfig::ImuType::Qmi8658) return false;
  // The FIFO is emptied and set with both sensors off, then both start at one rate.
  const bool enabled = writeReg(addr, QMI8658_REG_CTRL7, QMI8658_CTRL7_DISABLE_ALL) &&
                       writeReg(addr, QMI8658_REG_CTRL8, QMI8658_CTRL8_HANDSHAKE_STATUSINT) &&
                       qmi8658Ctrl9(addr, QMI8658_CTRL9_RST_FIFO) &&
                       writeReg(addr, QMI8658_REG_FIFO_CTRL, QMI8658_FIFO_CTRL_STREAM_128) &&
                       writeReg(addr, QMI8658_REG_CTRL2, QMI8658_CTRL2_FS_2G | QMI8658_CTRL2_ODR_224HZ) &&
                       writeReg(addr, QMI8658_REG_CTRL3, QMI8658_CTRL3_FS_512DPS | QMI8658_CTRL3_ODR_224HZ) &&
                       writeReg(addr, QMI8658_REG_CTRL7, QMI8658_CTRL7_ACC_GYRO_ENABLE);
  if (!enabled) qmi8658FifoOff(addr);
  return enabled;
}

bool Imu::disableFifo() {
  const uint8_t addr = addr_;
  if (!begun_ || addr == 0 || BoardConfig::ACTIVE.sensors.imuType != BoardConfig::ImuType::Qmi8658) return false;
  return qmi8658FifoOff(addr);
}

bool Imu::readFifo(const FifoSink sink, void* const context, uint16_t& frames) {
  frames = 0;
  const uint8_t addr = addr_;
  if (!begun_ || addr == 0 || BoardConfig::ACTIVE.sensors.imuType != BoardConfig::ImuType::Qmi8658) return false;
  // Datasheet 8.8: the count (in 2-byte words, FIFO_SMPL_CNT and FIFO_STATUS bits 1:0), read
  // mode through CTRL9, the data, then write mode again so the FIFO fills once more.
  uint8_t count[2] = {};
  if (!readRegs(addr, QMI8658_REG_FIFO_SMPL_CNT, count, sizeof(count))) return false;
  const uint16_t bytes = 2 * (((count[1] & 0x03) << 8) | count[0]);
  if (bytes == 0) return true;
  if (bytes % QMI8658_FIFO_FRAME_BYTES != 0) {
    // Not whole frames: reading on would pair every later frame wrongly.
    qmi8658Ctrl9(addr, QMI8658_CTRL9_RST_FIFO);
    return false;
  }
  const bool gap = (count[1] & (QMI8658_FIFO_STATUS_FULL | QMI8658_FIFO_STATUS_OVERFLOW)) != 0;
  bool ok = qmi8658Ctrl9(addr, QMI8658_CTRL9_REQ_FIFO);
  uint8_t raw[FIFO_CHUNK * QMI8658_FIFO_FRAME_BYTES];
  RawFrame chunk[FIFO_CHUNK];
  for (uint16_t left = bytes / QMI8658_FIFO_FRAME_BYTES; ok && left > 0;) {
    const uint8_t n = left < FIFO_CHUNK ? static_cast<uint8_t>(left) : FIFO_CHUNK;
    ok = readRegs(addr, QMI8658_REG_FIFO_DATA, raw, n * QMI8658_FIFO_FRAME_BYTES);
    if (!ok) break;
    for (uint8_t i = 0; i < n; ++i) {
      const uint8_t* b = &raw[i * QMI8658_FIFO_FRAME_BYTES];
      int16_t v[6];
      for (uint8_t j = 0; j < 6; ++j) v[j] = static_cast<int16_t>(b[2 * j] | b[2 * j + 1] << 8);
      chunk[i] = {v[0], v[1], v[2], v[3], v[4], v[5]};
    }
    sink(chunk, n, gap && frames == 0, context);
    frames += n;
    left -= n;
  }
  return writeReg(addr, QMI8658_REG_FIFO_CTRL, QMI8658_FIFO_CTRL_STREAM_128) && ok;
}

}  // namespace freeink

#else  // FREEINK_CAP_IMU - IMU absent.

namespace freeink {
bool Imu::begin() { return false; }
bool Imu::hasGyroscope() const { return false; }
bool Imu::read(Sample&) { return false; }
bool Imu::sleep() { return false; }
bool Imu::wake() { return false; }
bool Imu::enableFifo() { return false; }
bool Imu::disableFifo() { return false; }
bool Imu::readFifo(FifoSink, void*, uint16_t& frames) {
  frames = 0;
  return false;
}
}  // namespace freeink

#endif  // FREEINK_CAP_IMU
