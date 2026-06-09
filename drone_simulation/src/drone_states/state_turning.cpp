#include <memory>
#include "drone_states/state_turning.hpp"
#include "drone_states/state_accelerating.hpp"


StateTurning::StateTurning() {}

auto StateTurning::execute(DroneContext& context) -> std::unique_ptr<IDroneState> {
    float deltaAngle = normalizeAngle(context.directionToTarget - context.direction);
    context.direction = turnDrone(
        deltaAngle, context.config.angularSpeed, context.config.simTimeStep, context.direction 
    );

    if (std::fabs(deltaAngle) <= context.config.turnThreshold) {
        return std::make_unique<StateAccelerating>();
    }

    return nullptr;
}  

auto StateTurning::name() const -> const std::string {
    return "Turning";
} 
