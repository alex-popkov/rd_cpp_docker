#pragma once
#include "interfaces/drone_state.hpp"

class StateAccelerating : public IDroneState {
    public:
        StateAccelerating();

        auto execute(DroneContext& ctx) -> std::unique_ptr<IDroneState> override;  
        auto name() const -> const std::string override; 
};