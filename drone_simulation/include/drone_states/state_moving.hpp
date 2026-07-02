#pragma once
#include "interfaces/drone_state.hpp"

class StateMoving : public IDroneState {
    public:
        static constexpr const char* NAME = "MOVING";
        StateMoving();

        auto execute(DroneContext& context) -> std::unique_ptr<IDroneState> override;
        auto name() const -> std::string override;
};