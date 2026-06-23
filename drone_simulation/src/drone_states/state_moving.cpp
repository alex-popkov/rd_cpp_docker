#include <memory>
#include "drone_states/state_moving.hpp"
#include "drone_states/state_decelerating.hpp"
#include "simulation.hpp"

StateMoving::StateMoving() {}

auto StateMoving::execute(DroneContext& context) -> std::unique_ptr<IDroneState>
{
  float deltaAngle = normalizeAngle(context.directionToTarget - context.direction);

  if (fabs(deltaAngle) > context.config->turnThreshold) {
    return std::make_unique<StateDecelerating>();
  }

  return nullptr;
}

auto StateMoving::name() const -> DroneStates
{
  return DroneStates::Moving;
}