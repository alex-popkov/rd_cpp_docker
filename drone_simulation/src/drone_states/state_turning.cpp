
#include "drone_states/state_turning.hpp"
#include <memory>

StateTurning::StateTurning() {}

auto StateTurning::execute(DroneContext& ctx) -> std::unique_ptr<IDroneState> {
 return std::make_unique<StateTurning>();
};  

auto StateTurning::name() const -> const std::string {
    return "";
}; 
