#pragma once
#include "interfaces/drone_state.hpp"

class StateMoving : public IDroneState {
public:
  StateMoving();

  auto execute(const DroneStateInput& input) -> std::unique_ptr<IDroneState> override;
  auto name() const -> DroneStates override;
};