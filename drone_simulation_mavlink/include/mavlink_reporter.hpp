#pragma once
#include <cstdint>
#include <string>
#include "port_controllers/udp_port.hpp"
#include "drone_link.hpp"

class MavlinkReporter {
public:
  MavlinkReporter(const std::string& host, uint16_t port);

  auto sendHeartbeat() -> void;
  auto sendTelemetry(const dlink::Telemetry& t) -> void;  // GLOBAL_POSITION_INT + ATTITUDE

  auto sendDropCommand(double lat, double lon, float altM, uint8_t confirmation) -> void;
  auto pollDropAck() -> bool;

  static auto localToGps(float x, float y, double& lat, double& lon) -> void;

private:
  UdpPort udp;
};