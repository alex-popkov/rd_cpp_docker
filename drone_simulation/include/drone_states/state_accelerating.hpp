#pragma once
#include "interfaces/drone_state.hpp"

class StateAccelerating : public IDroneState {
    public:
        static constexpr const char* NAME = "ACCELERATING";
        StateAccelerating();

        auto execute(DroneContext& ctx) -> std::unique_ptr<IDroneState> override;
        auto name() const -> std::string override;
};