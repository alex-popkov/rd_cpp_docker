#include "drone_physics.hpp"
#include "drone_states/state_stopped.hpp"

DronePhysics::DronePhysics(const DroneConfig& config)
  : config(config)
{
  droneContext = {.directionToTarget = 0.0f,
                  .prevDirectionToTarget = 0.0f,
                  .speed = 0.0f,
                  .direction = config.initialDir,
                  .acceleration = config.attackSpeed * config.attackSpeed / (2.0f * config.accelPath),
                  .timeToStop = 0.0f,
                  .position = config.startPos,
                  .config = &this->config};

  state = std::make_unique<StateStopped>();
}

auto DronePhysics::stepPhysics(float dt) -> void
{
  DroneCommand cmd;
  if (commandQueue.tryPop(cmd)) {
    droneContext.directionToTarget = cmd.directionToTarget;
    droneContext.prevDirectionToTarget = cmd.prevDirectionToTarget;
  }

  auto nextState = state->execute(droneContext);
  if (nextState) {
    state = std::move(nextState);
  }

  elapsedTime += dt;
}

auto DronePhysics::sendCommand(const DroneCommand& cmd) -> void
{
  commandQueue.push(cmd);
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