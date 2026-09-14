#include "i2c_bus.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

I2cBus::I2cBus(const std::string& path)
{
  fd_ = ::open(path.c_str(), O_RDWR);
  if (fd_ < 0) {
    throw std::runtime_error("open(" + path + "): " + std::string(std::strerror(errno)));
  }
}

I2cBus::~I2cBus()
{
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

void I2cBus::select(int addr)
{
  if (::ioctl(fd_, I2C_SLAVE, addr) < 0) {
    throw std::runtime_error("ioctl(I2C_SLAVE): " + std::string(std::strerror(errno)));
  }
}

// Двофазний ЗАПИС: [reg, value] одним write.
void I2cBus::writeReg(std::uint8_t reg, std::uint8_t value)
{
  std::uint8_t buf[2] = {reg, value};

  // ssize_t write(int fd, const void* buf, size_t count);
  // fd_ — куди писати (дескриптор шини);
  // buf — звідки брати байти (вказівник на початок);
  // count — скільки байтів узяти з buf і відправити.

  auto count = ::write(fd_, buf, 2);
  if (count != 2) {
    throw std::runtime_error("writeReg: " + std::string(std::strerror(errno)));
  }
}

// Двофазне ЧИТАННЯ: write(reg) -> read(n байт). Ядро завдання.
void I2cBus::readRegs(std::uint8_t reg, std::uint8_t* value, std::size_t n)
{
  // Фаза 1: ставимо внутрішній вказівник чипа на reg.
  auto count = ::write(fd_, &reg, 1);
  if (count != 1) {
    throw std::runtime_error("readRegs write ptr: " + std::string(std::strerror(errno)));
  }

  // Фаза 2: читаємо дані. Перевіряємо, що прочитали рівно n.
  ssize_t got = ::read(fd_, value, n);
  if (got < 0) {
    throw std::runtime_error("readRegs read: " + std::string(std::strerror(errno)));
  }

  if (static_cast<std::size_t>(got) != n) {
    throw std::runtime_error("readRegs: обрив читання (" + std::to_string(got) + "/" + std::to_string(n) + " байт)");
  }
}

// Зручність для одного байта (наприклад WHO_AM_I).
std::uint8_t I2cBus::readReg8(std::uint8_t reg)
{
  std::uint8_t value = 0;
  readRegs(reg, &value, 1);
  return value;
}
