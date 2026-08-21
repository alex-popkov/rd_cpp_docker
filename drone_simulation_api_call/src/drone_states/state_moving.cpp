#include <memory>
#include "drone_states/state_moving.hpp"
#include "drone_states/state_decelerating.hpp"
#include "simulation.hpp"

StateMoving::StateMoving() {}

auto StateMoving::execute(const DroneStateInput& input) -> std::unique_ptr<IDroneState>
{
  if (fabs(input.deltaAngle) > input.config.turnThreshold) {
    return std::make_unique<StateDecelerating>();
  }

  return nullptr;
}

auto StateMoving::name() const -> DroneStates
{
  return DroneStates::Moving;
}