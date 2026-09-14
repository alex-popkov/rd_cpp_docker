#pragma once
#include "interfaces/drone_state.hpp"

class StateTurning : public IDroneState {
public:
  StateTurning();

  auto execute(const DroneStateInput& input) -> std::unique_ptr<IDroneState> override;
  auto name() const -> DroneStates override;
};