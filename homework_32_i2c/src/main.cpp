#include "i2c_bus.hpp"
#include "mpu6050.hpp"

#include <csignal>
#include <string>
#include <chrono>
#include <thread>
#include <format>
#include <iostream>

int main(int argc, char* argv[])
{
  const char* busPath = (argc > 1) ? argv[1] : "/dev/i2c-1";
  int addr = (argc > 2) ? std::stoi(argv[2], nullptr, 16) : 0x68;

  try {
    I2cBus bus(busPath);
    Mpu6050 mpu(bus, addr);
    mpu.begin();

    while (true) {
      Sample sample = mpu.read();
      std::cout << std::format(
        "\rAccel[g] {:+6.2f} {:+6.2f} {:+6.2f} | "
        "Gyro[dps] {:+7.1f} {:+7.1f} {:+7.1f} | T {:+5.1f}C   ",
        sample.ax,
        sample.ay,
        sample.az,
        sample.gx,
        sample.gy,
        sample.gz,
        sample.tempC);
      std::cout.flush();
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  }
  catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << std::endl;
    return 1;
  }
}