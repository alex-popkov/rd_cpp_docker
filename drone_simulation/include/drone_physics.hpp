#pragma once
#include <cstring>
#include <mutex>
#include <atomic>
#include "simulation.hpp"
#include "interfaces/drone_state.hpp"
#include "thread_safe_queue.hpp"

class DronePhysics {
  DroneContext droneContext;
  DroneConfig config;
  ThreadSafeQueue<DroneCommand> commandQueue;
  mutable std::mutex mtx;
  std::unique_ptr<IDroneState> state;
  float elapsedTime = 0.0f;

public:
  DronePhysics(const DroneConfig& config);

  auto stepPhysics(float dt) -> void;
  auto sendCommand(const DroneCommand& cmd) -> void;
  auto getTelemetry() const -> DroneTelemetry;
  auto getContext() const -> DroneContext;
  auto getStateName() const -> std::string;
};