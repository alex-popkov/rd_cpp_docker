#include <memory>
#include "drone_states/state_turning.hpp"
#include "drone_states/state_accelerating.hpp"
#include "simulation.hpp"

StateTurning::StateTurning() {}

auto StateTurning::execute(const DroneStateInput& input) -> std::unique_ptr<IDroneState>
{
  if (std::fabs(input.deltaAngle) <= input.config.turnThreshold) {
    return std::make_unique<StateAccelerating>();
  }

  return nullptr;
}

auto StateTurning::name() const -> DroneStates
{
  return DroneStates::Turning;
}
