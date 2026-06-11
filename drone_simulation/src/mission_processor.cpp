
#include <iostream>
#include "mission_processor.hpp"
#include "drone_states/state_stopped.hpp"

MissionProcessor::MissionProcessor(
    std::unique_ptr<ITargetProvider> targetProvider,
    std::unique_ptr<IBallisticSolver> solver
): targets(std::move(targetProvider)), ballisticSolver(std::move(solver)) {
}

auto MissionProcessor::init(std::unique_ptr<IConfigLoader> configLoader) -> void { 
    configLoader->load();
    droneConfig = configLoader->getConfig();
    ammo = configLoader->getAmmoParams();

    ammoFlightTime = getAmmoFlightTime(droneConfig, ammo);
    ammoHorizontalFlightDistance =  getAmmoHorizontalFlightDistance(
        droneConfig.attackSpeed,
        ammoFlightTime,
        ammo
    );
    droneContext = {
        .directionToTarget = 0.0f,   
        .prevDirectionToTarget = 0.0f,    
        .speed = 0.0f,
        .direction = droneConfig.initialDir,
        .acceleration = droneConfig.attackSpeed * droneConfig.attackSpeed / (2.0f * droneConfig.accelPath),
        .timeToStop = 0.0f,
        .position = droneConfig.startPos,
        .config = &droneConfig
    };
    state = std::make_unique<StateStopped>();
}

auto MissionProcessor::hasNext() -> bool { 
    return currentStep <= MAX_STEPS && !hit;
}

auto MissionProcessor::getCurrentStep() -> int { 
    return currentStep;
}

auto MissionProcessor::step() -> SimulationStep {
    int bestTargetIndex = -1;
    float bestTotalTime = INFINITY;
    Coord bestFireCoord = {
        .x = 0,
        .y = 0
    };

    //iteration by targets
    for (int i = 0; i < targets->getTargetCount(); ++i) {
        Coord target = getInterpolatedCoords(currentTime, droneConfig.arrayTimeStep, targets->getTarget(i));

        // balistic
        float distanceToTarget = getDistanceToTarget(target, droneContext.position);
        if (distanceToTarget < 1e-6f) {
            std::cout << "Distance to the target " << i << " is near or less then 0\n" << std::endl;
            continue;
        }
        float ratio = getRatio(distanceToTarget, ammoHorizontalFlightDistance);

        Coord fireCoord = getFireCoords(target, droneContext.position, ratio);
        float distanceToFire = getDistanceToTarget(fireCoord, droneContext.position);
        float droneToFireTime = distanceToFire / droneConfig.attackSpeed;
        float totalTimeRough = droneToFireTime + ammoFlightTime;

        Coord predictedTarget = getPredictedTargetCoords(
            currentTime, droneConfig.arrayTimeStep, targets->getTarget(i), target, totalTimeRough
        );

        //predicted balistic
        float newDistanceToTarget = getDistanceToTarget(predictedTarget, droneContext.position);
        if (newDistanceToTarget < 1e-6f) {
            std::cout << "Predicted distance to the target " << i << " is near or less then 0\n" << std::endl;
            continue;
        }

        float newRatio = getRatio(newDistanceToTarget, ammoHorizontalFlightDistance);
        Coord newFireCoord = getFireCoords(predictedTarget, droneContext.position, newRatio);
        float newDistanceToFire = getDistanceToTarget(newFireCoord, droneContext.position);
        float newDroneToFireTime = newDistanceToFire / droneConfig.attackSpeed;
        float newTotalTime = newDroneToFireTime + ammoFlightTime;
    
        if (newRatio < 0) {
            newTotalTime += getManeuveringTime(
                newDistanceToTarget, 
                ammoHorizontalFlightDistance,
                droneConfig.attackSpeed,
                droneContext.acceleration,
                droneConfig.angularSpeed
            );
        }

        if (prevTargetIndex != i && prevTargetIndex != -1) {
            newTotalTime += droneContext.timeToStop;
        }

        if (newTotalTime < bestTotalTime) {
            bestTotalTime = newTotalTime;
            bestTargetIndex = i;
            bestFireCoord = newFireCoord;
        }
    }


    if (bestTargetIndex < 0) {
        std::cout << "No valid target at this step, skipping"<< std::endl;
        ++currentStep;
        currentTime += droneConfig.simTimeStep;

        SimulationStep emptyStep = {
            .target          = -1,
            .dropPoint       = {0, 0},
            .aimPoint        = {0, 0},
            .predictedTarget = {0, 0},
            .droneMotion     = { droneContext.speed, droneContext.direction, droneContext.position, STOPPED }
        };

        return emptyStep;
    }

    Coord deltaToFire = bestFireCoord - droneContext.position;
    Coord dirVec = normalize(deltaToFire);
    droneContext.prevDirectionToTarget = prevDirToTarget;
    droneContext.directionToTarget = atan2(dirVec.y, dirVec.x);

    auto next = state->execute(droneContext);
    if (next) state = std::move(next);

    Coord bombLandCoord = getBombLandCoord(droneContext, ammoHorizontalFlightDistance);
    Coord targetAtImpact = getInterpolatedCoords(
        currentTime + ammoFlightTime, 
        droneConfig.arrayTimeStep,
        targets->getTarget(bestTargetIndex)
    );

    float bombMissDistance = getDistanceToTarget(bombLandCoord, targetAtImpact);
    Coord dir = { 
        .x = std::cos(droneContext.direction), 
        .y = std::sin(droneContext.direction) 
    };

    Coord aimPoint = droneContext.position + dir * ammoHorizontalFlightDistance;

    SimulationStep simulationStep = {
        .target = bestTargetIndex,
        .dropPoint = bestFireCoord,
        .aimPoint = aimPoint,
        .predictedTarget = targetAtImpact,
        .droneMotion = { droneContext.speed, droneContext.direction, droneContext.position, state->code() }
    };

    if (bombMissDistance < droneConfig.hitRadius) {
        std::cout << "Drone hit the target. Bomb miss: " << bombMissDistance << std::endl;
    
        hit = true;
    }

    if (bombLandCoord == targetAtImpact) {
        std::cout << "Direct hit"<< std::endl;

        hit = true;
    }

    prevTargetIndex = bestTargetIndex;
    prevDirToTarget = droneContext.directionToTarget; 

    currentStep++;
    currentTime += droneConfig.simTimeStep;

    return simulationStep;
}

auto MissionProcessor::reset() -> void {
    hit = false;
    prevTargetIndex = -1;
    prevDirToTarget = droneConfig.initialDir;
    currentStep = 1; 
    currentTime = 0.0f;

    droneContext = {
        .directionToTarget = 0.0f,
        .prevDirectionToTarget = 0.0f,
        .speed = 0.0f,
        .direction = droneConfig.initialDir,
        .acceleration = droneContext.acceleration,
        .timeToStop = 0.0f,
        .position = droneConfig.startPos,
        .config = &droneConfig
    };

    state = std::make_unique<StateStopped>();
}

auto MissionProcessor::changeSolver(std::unique_ptr<IBallisticSolver> ballisticSolver) -> void {
    this->ballisticSolver = std::move(ballisticSolver);
}
