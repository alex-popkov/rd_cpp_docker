#pragma once
#include "interfaces/drone_state.hpp"

class StateTurning : public IDroneState {
    public:
        static constexpr const char* NAME = "TURNING";
        StateTurning();

        auto execute(DroneContext& context) -> std::unique_ptr<IDroneState> override;
        auto name() const -> std::string override;
};