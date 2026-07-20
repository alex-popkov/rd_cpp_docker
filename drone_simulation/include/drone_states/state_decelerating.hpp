#pragma once
#include "interfaces/drone_state.hpp"

class StateDecelerating : public IDroneState {
public:
  StateDecelerating();

  auto execute(const DroneStateInput& input) -> std::unique_ptr<IDroneState> override;
  auto name() const -> DroneStates override;
};