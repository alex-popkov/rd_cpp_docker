#pragma once
#include "interfaces/drone_state.hpp"

class StateStopped : public IDroneState {
    public:
        StateStopped();

        auto execute(DroneContext& context) -> std::unique_ptr<IDroneState> override;  
        auto code() const -> DroneState override; 
};