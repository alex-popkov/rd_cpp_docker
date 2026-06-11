#pragma once
#include "interfaces/drone_state.hpp"

class StateDecelerating : public IDroneState {
    public:
        StateDecelerating();

        auto execute(DroneContext& context) -> std::unique_ptr<IDroneState> override;  
        auto code() const -> DroneState override; 
};