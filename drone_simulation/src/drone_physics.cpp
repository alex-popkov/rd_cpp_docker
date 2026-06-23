#include <thread>
#include "drone_physics.hpp"

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

auto DronePhysics::stepPhysics(float dt) -> void
{
  std::lock_guard<std::mutex> lock(mtx);
  DroneCommand cmd;
  if (this->commandQueue.tryPop(cmd)) {
    this->currentState = cmd.state;
    this->angleSpeed = cmd.angleSpeed;
  }

  this->processState(dt);

  this->elapsedTime += dt;
}

auto DronePhysics::processState(float dt) -> void
{
  switch (this->currentState) {
    case DroneStates::Stopped:
      break;

    case DroneStates::Turning:
      droneContext.direction += this->angleSpeed * dt;
      droneContext.direction = normalizeAngle(droneContext.direction);
      break;

    case DroneStates::Accelerating:
      droneContext.speed += droneContext.acceleration * dt;
      if (droneContext.speed > config.attackSpeed)
        droneContext.speed = config.attackSpeed;
      if (std::fabs(this->angleSpeed) > 1e-6f) {
        droneContext.direction += this->angleSpeed * dt;
        droneContext.direction = normalizeAngle(droneContext.direction);
      }
      droneContext.position = updateDronePosition(droneContext);
      break;

    case DroneStates::Moving:
      if (std::fabs(this->angleSpeed) > 1e-6f) {
        droneContext.direction += this->angleSpeed * dt;
        droneContext.direction = normalizeAngle(droneContext.direction);
      }
      droneContext.position = updateDronePosition(droneContext);
      break;

    case DroneStates::Decelerating:
      droneContext.speed -= droneContext.acceleration * dt;
      if (droneContext.speed < 0)
        droneContext.speed = 0;
      droneContext.position = updateDronePosition(droneContext);
      break;
  }
}