#include <memory>
#include "drone_states/state_stopped.hpp"
#include "drone_states/state_turning.hpp"
#include "drone_states/state_accelerating.hpp"
#include "simulation.hpp"

StateStopped::StateStopped() {}

auto StateStopped::execute(DroneContext& context) -> std::unique_ptr<IDroneState> 
{
    float deltaAngle = normalizeAngle(context.directionToTarget - context.direction);
    context.timeToStop = 0;

    if (std::fabs(deltaAngle) > context.config->turnThreshold) {

        return std::make_unique<StateTurning>();
    }
    
    return std::make_unique<StateAccelerating>();
}


auto StateStopped::name() const -> std::string
{
    return NAME;
}