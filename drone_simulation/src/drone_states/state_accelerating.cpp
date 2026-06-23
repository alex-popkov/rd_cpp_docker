#include <memory>
#include "drone_states/state_accelerating.hpp"
#include "drone_states/state_moving.hpp"
#include "simulation.hpp"

StateAccelerating::StateAccelerating() {}

auto StateAccelerating::execute(DroneContext& context) -> std::unique_ptr<IDroneState>
{
  if (context.speed >= context.config->attackSpeed) {
    return std::make_unique<StateMoving>();
  }

  return nullptr;
}

auto StateAccelerating::name() const -> DroneStates
{
  return DroneStates::Accelerating;
}