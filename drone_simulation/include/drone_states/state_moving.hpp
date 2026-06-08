#pragma once
#include "interfaces/drone_state.hpp"

class StateMoving : public IDroneState {
    public:
        StateMoving();

        auto execute(DroneContext& ctx) -> std::unique_ptr<IDroneState> override;  
        auto name() const -> const std::string override; 
};