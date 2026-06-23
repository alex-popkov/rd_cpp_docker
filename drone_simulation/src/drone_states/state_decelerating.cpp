#include <memory>
#include "drone_states/state_decelerating.hpp"
#include "drone_states/state_stopped.hpp"
#include "drone_states/state_accelerating.hpp"
#include "simulation.hpp"

StateDecelerating::StateDecelerating() {}

auto StateDecelerating::execute(DroneContext& context) -> std::unique_ptr<IDroneState>
{
  if (context.speed <= 1e-3f) {
    return std::make_unique<StateStopped>();
  }

  float deltaAngle = normalizeAngle(context.directionToTarget - context.direction);
  if (std::fabs(deltaAngle) <= context.config->turnThreshold) {
    return std::make_unique<StateAccelerating>();
  }

  return nullptr;
}

auto StateDecelerating::name() const -> DroneStates
{
  return DroneStates::Decelerating;
}