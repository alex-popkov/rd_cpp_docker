#include <memory>
#include "drone_states/state_stopped.hpp"
#include "drone_states/state_turning.hpp"
#include "drone_states/state_accelerating.hpp"
#include "simulation.hpp"

StateStopped::StateStopped() {}

auto StateStopped::execute(const DroneStateInput& input) -> std::unique_ptr<IDroneState>
{
  if (std::fabs(input.deltaAngle) > input.config.turnThreshold) {
    return std::make_unique<StateTurning>();
  }

  return std::make_unique<StateAccelerating>();
}

auto StateStopped::name() const -> DroneStates
{
  return DroneStates::Stopped;
}