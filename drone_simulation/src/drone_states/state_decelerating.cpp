#include <memory>
#include "drone_states/state_decelerating.hpp"
#include "drone_states/state_stopped.hpp"
#include "drone_states/state_accelerating.hpp"
#include "simulation.hpp"

StateDecelerating::StateDecelerating() {}

auto StateDecelerating::execute(const DroneStateInput& input) -> std::unique_ptr<IDroneState>
{
  if (input.speed <= 1e-3f) {
    return std::make_unique<StateStopped>();
  }

  if (std::fabs(input.deltaAngle) <= input.config.turnThreshold) {
    return std::make_unique<StateAccelerating>();
  }

  return nullptr;
}

auto StateDecelerating::name() const -> DroneStates
{
  return DroneStates::Decelerating;
}