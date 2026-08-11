#pragma once

#include <cstdint>
#include <string>

class I2cBus {
public:
  explicit I2cBus(const std::string& path);
  ~I2cBus();

  I2cBus(const I2cBus&) = delete;
  I2cBus& operator=(const I2cBus&) = delete;

  void select(int addr);
  void writeReg(std::uint8_t reg, std::uint8_t value);
  void readRegs(std::uint8_t reg, std::uint8_t* value, std::size_t n);
  std::uint8_t readReg8(std::uint8_t reg);

private:
  int fd_;
};