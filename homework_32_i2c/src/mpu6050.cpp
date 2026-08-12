#include "mpu6050.hpp"

#include <cstdint>
#include <format>
#include <stdexcept>

namespace {
// Регістри MPU-6050 (даташит)
constexpr std::uint8_t PWR_MGMT_1 = 0x6B;
constexpr std::uint8_t WHO_AM_I = 0x75;
constexpr std::uint8_t ACCEL_XOUT_H = 0x3B;
constexpr std::uint8_t EXPECTED_ID = 0x68;

// Чутливість (діапазони після ресету)
constexpr double ACCEL_LSB_PER_G = 16384.0;  // +-2g
constexpr double GYRO_LSB_PER_DPS = 131.0;   // +-250 deg/s

// Старший байт перший (big-endian).
std::int16_t be16(const std::uint8_t* p)
{
  return static_cast<std::int16_t>((p[0] << 8) | p[1]);
}
}  // namespace

Mpu6050::Mpu6050(I2cBus& bus, int addr)
  : bus_(bus)
  , addr_(addr)
{
}

void Mpu6050::begin()
{
  bus_.select(addr_);
  // Довести, що це саме MPU-6050.
  std::uint8_t who = bus_.readReg8(WHO_AM_I);

  if (who != EXPECTED_ID) {
    throw std::runtime_error(std::format("невiрний ID: очiкував 0x{:02X}, отримав 0x{:02X}", EXPECTED_ID, who));
  }

  // Розбудити з SLEEP.
  bus_.writeReg(PWR_MGMT_1, 0x00);
}

Sample Mpu6050::read()
{
  std::uint8_t buf[14];
  bus_.readRegs(ACCEL_XOUT_H, buf, sizeof(buf));

  // Порядок: Accel XYZ (6), Temp (2), Gyro XYZ (6)
  Sample s;
  s.ax = be16(&buf[0]) / ACCEL_LSB_PER_G;
  s.ay = be16(&buf[2]) / ACCEL_LSB_PER_G;
  s.az = be16(&buf[4]) / ACCEL_LSB_PER_G;
  s.tempC = be16(&buf[6]) / 340.0 + 36.53;
  s.gx = be16(&buf[8]) / GYRO_LSB_PER_DPS;
  s.gy = be16(&buf[10]) / GYRO_LSB_PER_DPS;
  s.gz = be16(&buf[12]) / GYRO_LSB_PER_DPS;

  return s;
}