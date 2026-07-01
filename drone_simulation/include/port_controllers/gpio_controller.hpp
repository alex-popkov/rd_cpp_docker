#pragma once
#include <gpiod.h>  // libgpiod — Linux API для роботи з GPIO

class GpioController {
public:
  GpioController(const char* chipName, int startLine, int dropLine);
  ~GpioController();

  auto signalStart() -> void;
  auto signalDrop() -> void;

private:
  gpiod_chip* chip;
  gpiod_line* startLine;
  gpiod_line* dropLine;
};