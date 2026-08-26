#include <iostream>
#include <cmath>
#include <cstring>
#include <vector>
#include <memory>
#include <chrono>
#include "mission_processor.hpp"
#include "config_loaders/factory.hpp"
#include "solvers/analytical_solver.hpp"
#include "port_controllers/uart_port.hpp"
#include "port_controllers/gpio_controller.hpp"
#include "drone_controller.hpp"
#include "interfaces/config_loader.hpp"
#include "log.hpp"
#include "mavlink_reporter.hpp"

auto main(int argc, char* argv[]) -> int
{
  try {
    std::string uartDev = "/tmp/ttyA";
    std::string gpioChip = "gpiochip0";
    std::string configPath = "config.json";
    std::string mavlinkDest = "127.0.0.1:14550";
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
      else if (arg == "--mavlink" && i + 1 < argc) {
        mavlinkDest = argv[++i];
      }
    }

    DEBUG("UART: " << uartDev << " GPIO: " << gpioChip << " START: " << startLine << " DROP: " << dropLine);

    auto loader = createLoader(LoaderType::FILE, configPath, "");
    loader->load();
    DroneConfig droneConfig = loader->getConfig();
    auto solver = std::make_unique<AnalyticalSolver>(droneConfig);
    MissionProcessor missionProcessor(std::move(solver), droneConfig);
    DroneController droneController(droneConfig);

    std::string mavHost = mavlinkDest.substr(0, mavlinkDest.find(':'));
    uint16_t mavPort = static_cast<uint16_t>(std::stoi(mavlinkDest.substr(mavlinkDest.find(':') + 1)));
    MavlinkReporter mavlinkReporter(mavHost, mavPort);

    DEBUG("Config: attackSpeed=" << droneConfig.attackSpeed << " accelPath=" << droneConfig.accelPath
                                 << " angularSpeed=" << droneConfig.angularSpeed);

    UartPort uart(uartDev.c_str());
    GpioController gpio(gpioChip.c_str(), startLine, dropLine);

    gpio.signalStart();
    LOG("START signal sent, waiting for data...");

    dlink::AmmoCfg ammo{};
    bool ammoReceived = false;

    std::vector<dlink::TargetPos> targetPositions;
    int targetCount = 0;

    dlink::Telemetry telemetry{};

    bool mpInitialized = false;

    dlink::Parser parser;
    uint8_t buffer[256];

    // mavlink heartbeat, first time
    auto lastHeartbeat = std::chrono::steady_clock::now();
    mavlinkReporter.sendHeartbeat();

    constexpr int MAX_ATTEMPTS = 5;
    constexpr auto ACK_TIMEOUT = std::chrono::milliseconds(500);

    bool mavlinkAcked = false;
    int mavlinkDropAttempts = 0;
    double dropLat = 0;
    double dropLon = 0;
    float dropAlt = 0;
    auto lastDropSend = std::chrono::steady_clock::now();

    bool running = true;
    bool dropSent = false;

    while (running) {
      // mavlink heartbeat
      auto now = std::chrono::steady_clock::now();
      if (now - lastHeartbeat >= std::chrono::seconds(1)) {
        mavlinkReporter.sendHeartbeat();
        lastHeartbeat = now;
      }

      // mavlink retry
      if (dropSent && !mavlinkAcked && mavlinkDropAttempts < MAX_ATTEMPTS) {
        if (mavlinkReporter.pollDropAck()) {
          mavlinkAcked = true;
          LOG("DROP ACK received");
        }
        else if (std::chrono::steady_clock::now() - lastDropSend >= ACK_TIMEOUT) {
          mavlinkReporter.sendDropCommand(dropLat, dropLon, dropAlt, mavlinkDropAttempts);
          mavlinkDropAttempts++;
          lastDropSend = std::chrono::steady_clock::now();
          LOG("DROP COMMAND_LONG attempt " << mavlinkDropAttempts);
        }
      }

      if (dropSent && (mavlinkAcked || mavlinkDropAttempts >= MAX_ATTEMPTS)) {
        if (!mavlinkAcked) {
          LOG("DROP ACK NOT received after " << MAX_ATTEMPTS << " attempts");
        }
        running = false;
      }

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
            mavlinkReporter.sendTelemetry(telemetry);

            DEBUG("t=" << telemetry.t_ms << " pos=(" << telemetry.x << "," << telemetry.y << ")" << " speed=" << telemetry.speed
                       << " dir=" << telemetry.dir << " z=" << telemetry.z << " state=" << (int)telemetry.state);

            if (ammoReceived && !mpInitialized) {
              missionProcessor.init(ammo, telemetry.z);
              mpInitialized = true;
            }

            dlink::Control control{0.0f, 0.0f};
            if (mpInitialized) {
              MissionResult result = missionProcessor.process(telemetry, targetPositions, targetCount);

              if (result.shouldDrop && !dropSent) {
                LOG("DROP!");
                gpio.signalDrop();

                // send drop mavlink
                MavlinkReporter::localToGps(telemetry.x, telemetry.y, dropLat, dropLon);
                dropAlt = telemetry.z;
                mavlinkReporter.sendDropCommand(dropLat, dropLon, dropAlt, 0);
                mavlinkDropAttempts = 1;
                lastDropSend = std::chrono::steady_clock::now();
                dropSent = true;
                LOG("DROP COMMAND_LONG attempt 1");
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
            LOG("AMMO: " << ammo.name << " mass=" << ammo.mass << " drag=" << ammo.drag << " lift=" << ammo.lift
                         << " hitRadius=" << ammo.hitRadius << " targets=" << (int)ammo.nTargets);
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
