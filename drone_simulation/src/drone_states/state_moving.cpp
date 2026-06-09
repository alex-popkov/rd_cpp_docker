#include <memory>
#include "drone_states/state_moving.hpp"
#include "drone_states/state_decelerating.hpp"


StateMoving::StateMoving() {}

auto StateMoving::execute(DroneContext& context) -> std::unique_ptr<IDroneState> {
    float deltaAngle = normalizeAngle(context.directionToTarget - context.direction);

    if (fabs(deltaAngle) > 1e-3f && fabs(deltaAngle) <= context.config.turnThreshold) {
        context.direction = turnDrone(deltaAngle, context.config.angularSpeed, context.config.simTimeStep, context.direction); 
    }

    context.position = updateDronePosition(context);

    if (fabs(deltaAngle) > context.config.turnThreshold) {
        return std::make_unique<StateDecelerating>();
    }

    return nullptr;
} 

auto StateMoving::name() const -> const std::string {
    return "Moving";
}