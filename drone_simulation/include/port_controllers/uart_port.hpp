#pragma once
#include <cstdint>
#include <cstddef>

class UartPort {
public:
  // device — шлях до порту: "/tmp/ttyA" (симуляція) або "/dev/ttyAMA1" (Raspberry Pi)
  explicit UartPort(const char* device);
  ~UartPort();

  // Прочитати доступні байти з порту. Повертає кількість прочитаних байтів,
  // або -1 якщо даних поки нема (бо порт відкритий з O_NONBLOCK).
  auto readBytes(uint8_t* buf, size_t maxLen) -> int;

  // Записати байти в порт — відправити дані на інший кінець.
  auto writeBytes(const uint8_t* buf, size_t len) -> void;

private:
  int fd;  // file descriptor — "номер" відкритого файлу в Linux
};