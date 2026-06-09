#include <memory>
#include "drone_states/state_decelerating.hpp"
#include "drone_states/state_stopped.hpp"
#include "drone_states/state_accelerating.hpp"

StateDecelerating::StateDecelerating() {}

auto StateDecelerating::execute(DroneContext& context) -> std::unique_ptr<IDroneState> {
    float deltaAngle = normalizeAngle(context.directionToTarget - context.direction);
    context.position = updateDronePosition(context);
    context.speed -= context.acceleration * context.config.simTimeStep;

    if (context.speed <= 1e-3f) {
        context.speed = 0;

        return std::make_unique<StateStopped>();
    }

    if (std::fabs(deltaAngle) <= context.config.turnThreshold) {
        return std::make_unique<StateAccelerating>();
    }

    return nullptr;
} 

auto StateDecelerating::name() const -> const std::string {
    return "Decelerating";
}