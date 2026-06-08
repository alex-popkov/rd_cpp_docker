#include "drone_states/state_moving.hpp"
#include <memory>

StateMoving::StateMoving() {}

auto StateMoving::execute(DroneContext& ctx) -> std::unique_ptr<IDroneState> {
 return std::make_unique<StateMoving>();
};  

auto StateMoving::name() const -> const std::string {
    return "";
}; 