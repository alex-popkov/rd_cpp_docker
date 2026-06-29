#pragma once
#include "simulation.hpp"
#include "drone_link.hpp"

class DroneController {
public:
  DroneController(const DroneConfig& config);
  auto computeControl(const DroneTelemetry& telemetry, const Coord& aimPoint) -> dlink::Control;

private:
  DroneConfig config;
  float prevDesiredDir = 0.0f;
  float prevTime = -1.0f;
  bool hasPrev = false;
};