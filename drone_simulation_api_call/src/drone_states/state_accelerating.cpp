#include <memory>
#include "drone_states/state_accelerating.hpp"
#include "drone_states/state_moving.hpp"
#include "simulation.hpp"

StateAccelerating::StateAccelerating() {}

auto StateAccelerating::execute(const DroneStateInput& input) -> std::unique_ptr<IDroneState>
{
  if (input.speed >= input.config.attackSpeed) {
    return std::make_unique<StateMoving>();
  }

  return nullptr;
}

auto StateAccelerating::name() const -> DroneStates
{
  return DroneStates::Accelerating;
}