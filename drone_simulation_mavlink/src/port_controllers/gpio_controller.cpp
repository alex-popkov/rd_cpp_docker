#include "port_controllers/gpio_controller.hpp"
#include <unistd.h>  // usleep()
#include <stdexcept>

GpioController::GpioController(const char* chipName, int startLine, int dropLine)
{
  // GPIO в Linux організовані в "чіпи" — кожен чіп має набір ліній (пінів).
  // На Raspberry Pi це gpiochip0 з ~30 лініями.
  // В симуляції gpio-sim створює свій чіп (gpiochip1, gpiochip2...).
  //
  // gpiod_chip_open_by_name() відкриває /dev/gpiochipN.
  this->chip = gpiod_chip_open_by_name(chipName);

  if (!this->chip) {
    throw std::runtime_error(std::string("Cannot open GPIO chip: ") + chipName);
  }

  // Отримати конкретну лінію за номером.
  // Лінія — це один GPIO пін. Номер відповідає фізичному або логічному піну.
  this->startLine = gpiod_chip_get_line(chip, startLine);
  this->dropLine = gpiod_chip_get_line(chip, dropLine);

  if (!this->startLine || !this->dropLine) {
    throw std::runtime_error("Cannot get GPIO lines");
  }

  // "Зарезервувати" лінію як OUTPUT.
  // В Linux лінією може керувати тільки один процес.
  // gpiod_line_request_output() робить три речі:
  //   1. Резервує лінію за нашим процесом (інші не зможуть її змінити)
  //   2. Встановлює напрямок — output (ми керуємо, а не читаємо)
  //   3. Виставляє початкове значення — 0 (LOW)
  // "drone" — ім'я споживача, видно в gpioinfo для відладки.
  gpiod_line_request_output(this->startLine, "drone", 0);
  gpiod_line_request_output(this->dropLine, "drone", 0);
}

GpioController::~GpioController()
{
  // Звільнити лінії — інші процеси зможуть їх використовувати.
  if (this->startLine) {
    gpiod_line_release(this->startLine);
  }

  if (this->dropLine) {
    gpiod_line_release(this->dropLine);
  }

  // Закрити чіп — звільнити file descriptor.
  if (this->chip) {
    gpiod_chip_close(this->chip);
  }
}

auto GpioController::signalStart() -> void
{
  // Виставити START = 1 (HIGH, 3.3V на реальній платі).
  // Чекер постійно читає цю лінію. Щойно бачить 1 — починає симуляцію:
  // шле PKT_AMMO, потім кожен такт PKT_TELEMETRY і PKT_TARGET.
  // Тримаємо START = 1 до кінця програми.

  gpiod_line_set_value(this->startLine, 1);
}

auto GpioController::signalDrop() -> void
{
  gpiod_line_set_value(this->dropLine, 1);
  usleep(80000);
  gpiod_line_set_value(this->dropLine, 0);
}