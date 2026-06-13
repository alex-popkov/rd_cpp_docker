#pragma once
#include "interfaces/drone_state.hpp"

class StateStopped : public IDroneState {
    public:
        static constexpr const char* NAME = "STOPPED";
        StateStopped();

        auto execute(DroneContext& context) -> std::unique_ptr<IDroneState> override;
        auto name() const -> std::string override;
};