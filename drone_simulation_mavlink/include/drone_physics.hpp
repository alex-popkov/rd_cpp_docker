#pragma once
#include <cstring>
#include <mutex>
#include <atomic>
#include "simulation.hpp"
#include "thread_safe_queue.hpp"

class DronePhysics {
public:
  DronePhysics(const DroneConfig& config);

  auto sendCommand(const DroneCommand& cmd) -> void;
  auto getTelemetry() const -> DroneTelemetry;
  auto run() -> void;
  auto start() -> void;
  auto stop() -> void;
  auto isThreadReady() const -> bool;

private:
  auto stepPhysics(float dt) -> void;
  auto processState(float dt) -> void;

  DroneContext droneContext;
  DroneConfig config;
  ThreadSafeQueue<DroneCommand> commandQueue;
  mutable std::mutex mtx;
  float elapsedTime = 0.0f;
  float angleSpeed = 0.0f;
  DroneStates currentState = DroneStates::Stopped;
  std::atomic<bool> ready{false};
  std::atomic<bool> running{false};
  std::atomic<bool> stopFlag{false};
};
