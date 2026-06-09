#include <memory>
#include "drone_states/state_accelerating.hpp"
#include "drone_states/state_moving.hpp"


StateAccelerating::StateAccelerating() {}

auto StateAccelerating::execute(DroneContext& context) -> std::unique_ptr<IDroneState> {

    float deltaAngle = normalizeAngle(context.directionToTarget - context.direction);
    context.speed += context.acceleration * context.config.simTimeStep;
    
    if (context.speed > context.config.attackSpeed) {
        context.speed = context.config.attackSpeed;
    }

    if (fabs(deltaAngle) > 1e-3f && fabs(deltaAngle) <= context.config.turnThreshold) {
        context.direction = turnDrone(deltaAngle, context.config.angularSpeed, context.config.simTimeStep, context.direction); 
    }

    context.position = updateDronePosition(context);

    if (context.speed >= context.config.attackSpeed) {
        return std::make_unique<StateMoving>();
    }

    return nullptr;
}  

auto StateAccelerating::name() const -> const std::string {
    return "Accelerating";
}