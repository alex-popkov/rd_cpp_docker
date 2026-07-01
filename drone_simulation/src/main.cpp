#include <iostream>
#include <cmath>
#include <cstring>
#include <vector>
#include <memory>
#include "mission_processor.hpp"
#include "config_loaders/factory.hpp"
#include "solvers/analytical_solver.hpp"
#include "port_controllers/uart_port.hpp"
#include "port_controllers/gpio_controller.hpp"
#include "drone_controller.hpp"
#include "interfaces/config_loader.hpp"

auto main(int argc, char* argv[]) -> int
{
  try {
    std::string uartDev = "/tmp/ttyA";
    std::string gpioChip = "gpiochip0";
    std::string configPath = "config.json";
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
      else if (arg == "--config" && i + 1 < argc) {
        configPath = argv[++i];
      }
    }

    std::cout << "UART: " << uartDev << " GPIO: " << gpioChip << " START: " << startLine << " DROP: " << dropLine << std::endl;

    auto loader = createLoader(LoaderType::FILE, configPath, "");
    loader->load();
    DroneConfig droneConfig = loader->getConfig();
    auto solver = std::make_unique<AnalyticalSolver>(droneConfig);
    MissionProcessor missionProcessor(std::move(solver), droneConfig);
    DroneController droneController(droneConfig);

    std::cout << "Config: attackSpeed=" << droneConfig.attackSpeed << " accelPath=" << droneConfig.accelPath
              << " angularSpeed=" << droneConfig.angularSpeed << std::endl;

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
                      << " dir=" << telemetry.dir << " z=" << telemetry.z << " state=" << (int)telemetry.state << std::endl;

            if (ammoReceived && !mpInitialized) {
              std::cout << "MP init with altitude=" << telemetry.z << std::endl;
              missionProcessor.init(ammo, telemetry.z);
              mpInitialized = true;
            }

            dlink::Control control{0.0f, 0.0f};
            if (mpInitialized) {
              MissionResult result = missionProcessor.process(telemetry, targetPositions, targetCount);

              if (result.shouldDrop && !dropped) {
                std::cout << "DROP!" << std::endl;
                gpio.signalDrop();
                dropped = true;
              }

              DroneTelemetry droneTelemetry = mapDlinkTelemetry(telemetry);
              control = droneController.computeControl(droneTelemetry, result.aimPoint);
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
