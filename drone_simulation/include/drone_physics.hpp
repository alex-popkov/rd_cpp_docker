#pragma once
#include <cstring>
#include <mutex>
#include <atomic>
#include "simulation.hpp"
#include "interfaces/drone_state.hpp"
#include "thread_safe_queue.hpp"

class DronePhysics {
public:
  DronePhysics(const DroneConfig& config);

  auto sendCommand(const DroneCommand& cmd) -> void;
  auto getTelemetry() const -> DroneTelemetry;
  auto getContext() const -> DroneContext;
  auto getStateName() const -> std::string;
  auto run() -> void;
  auto start() -> void;
  auto stop() -> void;
  auto isThreadReady() const -> bool;

private:
  auto stepPhysics(float dt) -> void;

  DroneContext droneContext;
  DroneConfig config;
  ThreadSafeQueue<DroneCommand> commandQueue;
  mutable std::mutex mtx;
  std::unique_ptr<IDroneState> state;
  float elapsedTime = 0.0f;
  std::atomic<bool> ready{false};
  std::atomic<bool> running{false};
  std::atomic<bool> stopFlag{false};
};
