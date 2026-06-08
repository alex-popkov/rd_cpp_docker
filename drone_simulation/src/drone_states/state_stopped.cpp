#include "drone_states/state_stopped.hpp"
#include <memory>

StateStopped::StateStopped() {}

auto StateStopped::execute(DroneContext& ctx) -> std::unique_ptr<IDroneState> {
 return std::make_unique<StateStopped>();
};  

auto StateStopped::name() const -> const std::string {
    return "";
}; 