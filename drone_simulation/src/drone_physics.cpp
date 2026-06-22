#include <thread>
#include "drone_physics.hpp"
#include "drone_states/state_stopped.hpp"

DronePhysics::DronePhysics(const DroneConfig& config)
  : config(config)
{
  this->droneContext = {.directionToTarget = 0.0f,
                        .prevDirectionToTarget = 0.0f,
                        .speed = 0.0f,
                        .direction = config.initialDir,
                        .acceleration = config.attackSpeed * config.attackSpeed / (2.0f * config.accelPath),
                        .timeToStop = 0.0f,
                        .position = config.startPos,
                        .config = &this->config};

  this->state = std::make_unique<StateStopped>();
}

auto DronePhysics::stepPhysics(float dt) -> void
{
  std::lock_guard<std::mutex> lock(mtx);
  DroneCommand cmd;
  if (this->commandQueue.tryPop(cmd)) {
    this->droneContext.directionToTarget = cmd.directionToTarget;
    this->droneContext.prevDirectionToTarget = cmd.prevDirectionToTarget;
  }

  auto nextState = this->state->execute(droneContext);
  if (nextState) {
    this->state = std::move(nextState);
  }

  this->elapsedTime += dt;
}

auto DronePhysics::sendCommand(const DroneCommand& cmd) -> void
{
  this->commandQueue.push(cmd);
}

auto DronePhysics::getTelemetry() const -> DroneTelemetry
{
  std::lock_guard<std::mutex> lock(mtx);

  return {.pos = droneContext.position, .speed = droneContext.speed, .direction = droneContext.direction, .timeSecSinceStart = elapsedTime};
}

auto DronePhysics::getContext() const -> DroneContext
{
  std::lock_guard<std::mutex> lock(mtx);

  return droneContext;
}

auto DronePhysics::getStateName() const -> std::string
{
  std::lock_guard<std::mutex> lock(mtx);

  return state->name();
}

auto DronePhysics::run() -> void
{
  this->ready.store(true);

  while (!this->running.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  while (!this->stopFlag.load()) {
    this->stepPhysics(this->config.physicsTimeStep);
    std::this_thread::sleep_for(std::chrono::duration<float>(this->config.physicsTimeStep / this->config.timeScale));
  }
}

auto DronePhysics::start() -> void
{
  this->running.store(true);
}

auto DronePhysics::stop() -> void
{
  this->stopFlag.store(true);
}

auto DronePhysics::isThreadReady() const -> bool
{
  return this->ready.load();
}