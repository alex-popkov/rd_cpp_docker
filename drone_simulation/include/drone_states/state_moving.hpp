#pragma once
#include "interfaces/drone_state.hpp"

class StateMoving : public IDroneState {
    public:
        StateMoving();

        auto execute(DroneContext& context) -> std::unique_ptr<IDroneState> override;  
        auto code() const -> DroneState override; 
};