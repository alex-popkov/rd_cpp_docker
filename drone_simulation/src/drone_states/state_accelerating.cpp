#include "drone_states/state_accelerating.hpp"
#include <memory>

StateAccelerating::StateAccelerating() {}

auto StateAccelerating::execute(DroneContext& ctx) -> std::unique_ptr<IDroneState> {
 return std::make_unique<StateAccelerating>();
};  

auto StateAccelerating::name() const -> const std::string {
    return "";
}; 