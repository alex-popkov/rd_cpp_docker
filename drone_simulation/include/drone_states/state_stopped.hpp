#pragma once
#include "interfaces/drone_state.hpp"

class StateStopped : public IDroneState {
public:
  StateStopped();

  auto execute(const DroneStateInput& input) -> std::unique_ptr<IDroneState> override;
  auto name() const -> DroneStates override;
};