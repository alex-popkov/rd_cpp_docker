#pragma once
#include <cstdint>
#include <cstddef>

class UartPort {
public:
  explicit UartPort(const char* device);
  ~UartPort();

  auto readBytes(uint8_t* buffer, size_t maxByteCount) -> int;
  auto writeBytes(const uint8_t* buffer, size_t byteCount) -> void;

private:
  int fd;
};