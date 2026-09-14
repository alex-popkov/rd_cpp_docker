#pragma once

#include "i2c_bus.hpp"

struct Sample {
  double ax, ay, az;
  double gx, gy, gz;
  double tempC;
};

class Mpu6050 {
public:
  Mpu6050(I2cBus& bus, int addr);

  void begin();
  Sample read();

private:
  I2cBus& bus_;
  int addr_;
};