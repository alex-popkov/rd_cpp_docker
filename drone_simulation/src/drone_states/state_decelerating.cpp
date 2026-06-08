#include "drone_states/state_decelerating.hpp"
#include <memory>

StateDecelerating::StateDecelerating() {}

auto StateDecelerating::execute(DroneContext& ctx) -> std::unique_ptr<IDroneState> {
 return std::make_unique<StateDecelerating>();
};  

auto StateDecelerating::name() const -> const std::string {
    return "";
}; 