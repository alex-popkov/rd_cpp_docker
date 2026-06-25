#include <fcntl.h>    // open(), O_RDWR, O_NOCTTY, O_NONBLOCK
#include <termios.h>  // termios, cfmakeraw, cfsetispeed, tcsetattr
#include <unistd.h>   // read(), write(), close()
#include <cstring>
#include <stdexcept>
#include "port_controllers/uart_port.hpp"

UartPort::UartPort(const char* device)
{
  // Відкриваємо порт як файл.
  // O_RDWR     — і читати, і писати
  // O_NOCTTY   — не робити цей порт контрольним терміналом процесу
  //              (інакше Ctrl+C з порту вб'є програму)
  // O_NONBLOCK — read() не блокує: якщо даних нема, одразу повертає -1
  //              замість того щоб чекати. Це важливо для головного циклу —
  //              ми не хочемо зависати на read(), бо потрібно і читати, і слати.
  this->fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (this->fd < 0) {
    throw std::runtime_error("Cannot open UART: " + std::string(device) + "\n");
  }

  // termios — структура, що описує налаштування послідовного порту.
  // Кожен порт має свої параметри: швидкість, кількість біт, парність тощо.
  termios tio{};
  tcgetattr(this->fd, &tio);  // прочитати поточні налаштування порту

  // cfmakeraw() — вимкнути всю "термінальну" обробку.
  // Без цього Linux намагається інтерпретувати дані як текст:
  // обробляє Enter, Backspace, Ctrl+C, ехо тощо.
  // Нам потрібні сирі байти без жодних перетворень — raw mode.
  // Також виставляє 8 біт даних, без парності (8N1).
  cfmakeraw(&tio);

  // Швидкість передачі — 115200 біт/с (baud rate).
  // Обидва кінці UART мусять мати однакову швидкість,
  // інакше дані будуть сміттям. 115200 — стандартна для embedded.
  cfsetispeed(&tio, B115200);  // вхідна швидкість (receive)
  cfsetospeed(&tio, B115200);  // вихідна швидкість (transmit)

  // CLOCAL — ігнорувати сигнали модему (DCD/DSR). У нас немає модему.
  // CREAD  — увімкнути приймач. Без цього read() нічого не поверне.
  tio.c_cflag |= (CLOCAL | CREAD);

  // Застосувати налаштування негайно (TCSANOW = "зараз").
  // Альтернатива TCSADRAIN чекала б поки все з буфера відправиться.
  tcsetattr(this->fd, TCSANOW, &tio);
}

UartPort::~UartPort()
{
  if (this->fd >= 0) {
    close(this->fd);
  }
}

auto UartPort::readBytes(uint8_t* buf, size_t maxLen) -> int
{
  // read() поверне:
  //   > 0  — кількість прочитаних байтів
  //   -1   — помилка або "даних нема" (errno == EAGAIN, бо O_NONBLOCK)
  // В головному циклі ми просто ігноруємо -1 і пробуємо знову.
  return read(this->fd, buf, maxLen);
}

auto UartPort::writeBytes(const uint8_t* buf, size_t len) -> void
{
  // write() кладе байти в буфер ядра, звідки вони підуть на дріт.
  // Для коротких пакетів (8-16 байт) це відбувається миттєво.
  write(this->fd, buf, len);
}