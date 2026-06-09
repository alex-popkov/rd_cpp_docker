#pragma once
#include "interfaces/drone_state.hpp"

class StateStopped : public IDroneState {
    public:
        StateStopped();

        auto execute(DroneContext& context) -> std::unique_ptr<IDroneState> override;  
        auto name() const -> const std::string override; 
};