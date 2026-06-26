#include <iostream>
#include <cmath>
#include <cstring>
#include <cstring>
#include <vector>
#include "simulation.hpp"
#include "mission_processor.hpp"
#include "drone_link.hpp"
#include "port_controllers/uart_port.hpp"
#include "port_controllers/gpio_controller.hpp"

using json = nlohmann::json;

#define ENABLE_LOG 1
#define ENABLE_DEBUG 0

#if ENABLE_LOG
#define LOG(msg) std::cout << "[LOG] " << msg << std::endl
#else
#define LOG(msg)
#endif

#if ENABLE_DEBUG
#define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
#define DEBUG(msg)
#endif

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

    dlink::Parser parser;
    uint8_t buffer[256];

    while (!dropped) {
      int n = uart.readBytes(buffer, sizeof(buffer));

      if (n <= 0) {
        continue;
      }

      uint8_t type, len, payload[260];
      for (int i = 0; i < n; i++) {
        // feed() повертає true коли зібрано повний валідний кадр
        if (!parser.feed(buffer[i], type, payload, len)) {
          continue;
        }

        switch (type) {
          case dlink::PKT_TELEMETRY: {
            std::memcpy(&telemetry, payload, sizeof(telemetry));

            std::cout << "t=" << telemetry.t_ms << " pos=(" << telemetry.x << "," << telemetry.y << ")" << " speed=" << telemetry.speed
                      << " dir=" << telemetry.dir << std::endl;

            dlink::Control control{0.0f, 0.0f};
            uint8_t out[64];
            size_t m = encode(dlink::PKT_CONTROL, &control, sizeof(control), out);
            uart.writeBytes(out, m);
            break;
          }

          case dlink::PKT_AMMO: {
            // Приходить один раз на старті.
            // Містить параметри боєприпасу і кількість цілей.
            std::memcpy(&ammo, payload, sizeof(ammo));
            ammoReceived = true;
            targetCount = ammo.nTargets;
            std::cout << "AMMO: " << ammo.name << " mass=" << ammo.mass << " drag=" << ammo.drag << " lift=" << ammo.lift
                      << " hitRadius=" << ammo.hitRadius << " targets=" << (int)ammo.nTargets << std::endl;
            break;
          }

          case dlink::PKT_TARGET: {
            // Приходить періодично для кожної цілі.
            // Оновлюємо позицію за id.
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

    // SolverType solverType = (solverArg == "analytical") ? SolverType::ANALYTICAL : SolverType::TABLE;
    // auto ballisticSolver = createSolver(solverType, droneConfig, ballisticTablePath);

    // MissionProcessor missionProcessor(std::move(targets), std::move(ballisticSolver), physicsPtr);
    // missionProcessor.init(std::move(loader));

    // writeSimulationJSONFile(missionProcessor.getSteps());

    return 0;
  }
  catch (const std::exception& error) {
    LOG("Error: " << error.what());

    return 1;
  }
}
