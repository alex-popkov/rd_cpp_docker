#pragma once
#include "interfaces/drone_state.hpp"

class StateTurning : public IDroneState {
    public:
        StateTurning();

        auto execute(DroneContext& context) -> std::unique_ptr<IDroneState> override;  
        auto name() const -> const std::string override; 
};