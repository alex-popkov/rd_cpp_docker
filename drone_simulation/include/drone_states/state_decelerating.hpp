#pragma once
#include "interfaces/drone_state.hpp"

class StateDecelerating : public IDroneState {
    public:
        static constexpr const char* NAME = "DECELERATING";
        StateDecelerating();

        auto execute(DroneContext& context) -> std::unique_ptr<IDroneState> override;
        auto name() const -> std::string override;
};