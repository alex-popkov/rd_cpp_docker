#include <iostream>
#include <cmath>
#include <cstring>
#include <vector>
#include <memory>
#include "mission_processor.hpp"
#include "solvers/analytical_solver.hpp"
#include "port_controllers/uart_port.hpp"
#include "port_controllers/gpio_controller.hpp"

auto main(int argc, char* argv[]) -> int
{
  try {
    std::string uartDev = "/tmp/ttyA";
    std::string gpioChip = "gpiochip0";
    int startLine = 24;
    int dropLine = 23;

    for (int i = 1; i < argc; i++) {
      std::string arg = argv[i];
      if (arg == "--uart" && i + 1 < argc) {
        uartDev = argv[++i];
      }
      else if (arg == "--gpiochip" && i + 1 < argc) {
        gpioChip = argv[++i];
      }
      else if (arg == "--start-line" && i + 1 < argc) {
        startLine = std::stoi(argv[++i]);
      }
      else if (arg == "--drop-line" && i + 1 < argc) {
        dropLine = std::stoi(argv[++i]);
      }
    }

    std::cout << "UART: " << uartDev << " GPIO: " << gpioChip << " START: " << startLine << " DROP: " << dropLine << std::endl;

    UartPort uart(uartDev.c_str());
    GpioController gpio(gpioChip.c_str(), startLine, dropLine);

    gpio.signalStart();
    std::cout << "START signal sent, waiting for data..." << std::endl;

    dlink::AmmoCfg ammo{};
    bool ammoReceived = false;

    std::vector<dlink::TargetPos> targetPositions;
    int targetCount = 0;

    dlink::Telemetry telemetry{};

    bool dropped = false;
    bool mpInitialized = false;

    std::unique_ptr<MissionProcessor> missionProcessor;

    dlink::Parser parser;
    uint8_t buffer[256];

    while (!dropped) {
      int n = uart.readBytes(buffer, sizeof(buffer));

      if (n <= 0) {
        continue;
      }

      uint8_t type, len, payload[260];
      for (int i = 0; i < n; i++) {
        if (!parser.feed(buffer[i], type, payload, len)) {
          continue;
        }

        switch (type) {
          case dlink::PKT_TELEMETRY: {
            std::memcpy(&telemetry, payload, sizeof(telemetry));

            std::cout << "t=" << telemetry.t_ms << " pos=(" << telemetry.x << "," << telemetry.y << ")" << " speed=" << telemetry.speed
                      << " dir=" << telemetry.dir << std::endl;

            // Створити солвер і MP після першої телеметрії — тепер знаємо altitude і speed
            if (ammoReceived && !mpInitialized) {
              float speed = telemetry.speed > 1.0f ? telemetry.speed : 30.0f;

              DroneConfig solverConfig{};
              solverConfig.altitude = telemetry.z;
              solverConfig.attackSpeed = speed;

              auto solver = std::make_unique<AnalyticalSolver>(solverConfig);
              missionProcessor = std::make_unique<MissionProcessor>(std::move(solver));
              missionProcessor->init(ammo, telemetry.z, speed);
              mpInitialized = true;
            }

            // Отримати рішення від MP
            dlink::Control control{0.0f, 0.0f};
            if (mpInitialized) {
              MissionResult result = missionProcessor->process(telemetry, targetPositions, targetCount);

              if (result.shouldDrop && !dropped) {
                std::cout << "DROP!" << std::endl;
                gpio.signalDrop();
                dropped = true;
              }

              control = result.control;
            }

            uint8_t out[64];
            size_t m = dlink::encode(dlink::PKT_CONTROL, &control, sizeof(control), out);
            uart.writeBytes(out, m);
            break;
          }

          case dlink::PKT_AMMO: {
            std::memcpy(&ammo, payload, sizeof(ammo));
            ammoReceived = true;
            targetCount = ammo.nTargets;
            targetPositions.resize(targetCount);
            std::cout << "AMMO: " << ammo.name << " mass=" << ammo.mass << " drag=" << ammo.drag << " lift=" << ammo.lift
                      << " hitRadius=" << ammo.hitRadius << " targets=" << (int)ammo.nTargets << std::endl;
            break;
          }

          case dlink::PKT_TARGET: {
            dlink::TargetPos targetPosition;
            std::memcpy(&targetPosition, payload, sizeof(targetPosition));

            if (targetPosition.id < targetPositions.size()) {
              targetPositions[targetPosition.id] = targetPosition;
            }
            break;
          }
        }
      }
    }

    return 0;
  }
  catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << std::endl;
    return 1;
  }
}
