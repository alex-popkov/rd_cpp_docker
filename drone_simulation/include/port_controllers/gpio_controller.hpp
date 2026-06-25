#pragma once
#include <gpiod.h>  // libgpiod — Linux API для роботи з GPIO

class GpioController {
public:
  // chipName  — ім'я GPIO контролера: "gpiochip0" (Raspberry Pi) або "gpiochip1" (симуляція)
  // startLine — номер піна для сигналу START (24 за замовчуванням)
  // dropLine  — номер піна для сигналу DROP (23 за замовчуванням)
  GpioController(const char* chipName, int startLine, int dropLine);
  ~GpioController();

  // Підняти START у 1 — сигнал чекеру "я готовий, починай симуляцію"
  auto signalStart() -> void;

  // Імпульс DROP: 1 → чекати 80мс → 0. Чекер ловить фронт і рахує балістику.
  auto signalDrop() -> void;

private:
  gpiod_chip* chip;       // "чіп" — контролер, що містить набір GPIO ліній
  gpiod_line* startLine;  // конкретна лінія (пін) для START
  gpiod_line* dropLine;   // конкретна лінія (пін) для DROP
};