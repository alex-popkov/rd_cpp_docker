#include <iostream>
#include <cmath>
#include <cstring>
#include "../include/json.hpp"
#include "../include/simulation.hpp"

using json = nlohmann::json;

#define ENABLE_LOG	1
#define ENABLE_DEBUG  0
 
#if ENABLE_LOG
  #define LOG(msg) std::cout << "[LOG] " << msg << std::endl
#else
  #define LOG(msg)
#endif
 
#if ENABLE_DEBUG
  #define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
  #define DEBUG(msg)
#endif


int main() {
    const float g = 9.81f;

    DroneConfig droneConfig;
    json ammoJSON;
    AmmoParams* ammoArr = nullptr;
    AmmoParams ammo;
    json targetsJSON;
    Coord** targets = nullptr;
    float ammoFlightTime;
    float ammoHorizontalFlightDistance;
    int targetsCount;
    int ammoCount;
    int timeSteps;

    try {
        droneConfig = readDroneConfig("config.json");
        ammoJSON = parseJSONfile("ammo.json");
        ammoCount = ammoJSON.size();
        ammoArr = readAmmo(ammoJSON);
        ammo = findAmmo(ammoArr, droneConfig.ammoName, ammoCount);
        delete[] ammoArr;
        ammoArr = nullptr;
        targetsJSON = parseJSONfile("targets.json");
        targetsCount = targetsJSON["targetCount"];
        timeSteps = targetsJSON["timeSteps"];
        targets = fillTargets(targetsJSON);
        ammoFlightTime = getAmmoFlightTime(droneConfig, ammo, g);
        ammoHorizontalFlightDistance =  getAmmoHorizontalFlightDistance(
            droneConfig.attackSpeed,
            ammoFlightTime,
            ammo,
            g
        );
    } catch (const std::exception& error) {
        LOG("Error: " << error.what());
        freeTargets(targets, targetsCount);

        return 1;
    }

    //simulation start
    const int MAX_STEPS = 10000;
    const float acceleration = droneConfig.attackSpeed * droneConfig.attackSpeed / (2.0f * droneConfig.accelPath);
    int currentStep = 1;
    float currentTime = 0.0f;
    SimulationStep* simSteps = new SimulationStep[MAX_STEPS + 1];
    DroneMotion droneMotion = {
        .speed = 0.0f,
        .dir = droneConfig.initialDir,
        .pos = {
            .x = droneConfig.startPos.x,
            .y = droneConfig.startPos.y
        },
        .state = STOPPED
    };
    int prevTargetIndex = -1;
    float prevDirToTarget = droneConfig.initialDir;

    //write initial sim data
    simSteps[0] = {
        .target = -1,
        .droneMotion = {
            .dir = droneConfig.initialDir,
            .pos = droneConfig.startPos,
            .state = STOPPED
        }
    };

    //simulation loop
    while(true) {
        if (currentStep > MAX_STEPS) {
            LOG("The maximum number of steps has been reached\n");
            freeTargets(targets, targetsCount); 

            return 1;
        }

        int bestTargetIndex = -1;
        float bestTotalTime = INFINITY;
        Coord bestFireCoord = {
            .x = 0,
            .y = 0
        };

        //iteration by targets
        for (int i = 0; i < targetsCount; ++i) {
            Coord target = getInterpolatedCoords(currentTime, droneConfig.arrayTimeStep, targets[i], timeSteps);

            // balistic
            float distanceToTarget = getDistanceToTarget(target, droneMotion.pos);
            if (distanceToTarget < 1e-6f) {
                LOG("Distance to the target " << i << " is near or less then 0\n");
                continue;
            }
            float ratio = getRatio(distanceToTarget, ammoHorizontalFlightDistance);

            Coord fireCoord = getFireCoords(target, droneMotion.pos, ratio);
            float distanceToFire = getDistanceToTarget(fireCoord, droneMotion.pos);
            float droneToFireTime = distanceToFire / droneConfig.attackSpeed;
            float totalTimeRough = droneToFireTime + ammoFlightTime;

            Coord predictedTarget = getPredictedTargetCoords(
                currentTime, droneConfig.arrayTimeStep, targets[i], target, totalTimeRough, timeSteps
            );

            //predicted balistic
            float newDistanceToTarget = getDistanceToTarget(predictedTarget, droneMotion.pos);
            if (newDistanceToTarget < 1e-6f) {
                LOG("Predicted distance to the target " << i << " is near or less then 0\n");
                continue;
            }

            float newRatio = getRatio(newDistanceToTarget, ammoHorizontalFlightDistance);
            Coord newFireCoord = getFireCoords(predictedTarget, droneMotion.pos, newRatio);
            float newDistanceToFire = getDistanceToTarget(newFireCoord, droneMotion.pos);
            float newDroneToFireTime = newDistanceToFire / droneConfig.attackSpeed;
            float newTotalTime = newDroneToFireTime + ammoFlightTime;
          
            if (newRatio < 0) {
                newTotalTime += getManeuveringTime(
                    newDistanceToTarget, 
                    ammoHorizontalFlightDistance,
                    droneConfig.attackSpeed,
                    acceleration,
                    droneConfig.angularSpeed
                );
            }

            if (prevTargetIndex != i && prevTargetIndex != -1) {
                newTotalTime += getTimeToStop(
                    droneConfig.attackSpeed, 
                    acceleration,  
                    prevDirToTarget, 
                    droneConfig.angularSpeed,
                    droneMotion
                );
            }

            if (newTotalTime < bestTotalTime) {
                bestTotalTime = newTotalTime;
                bestTargetIndex = i;
                bestFireCoord = newFireCoord;
            }
        }


        if (bestTargetIndex < 0) {
            LOG("No valid target at this step, skipping");
            ++currentStep;
            currentTime += droneConfig.simTimeStep;

            continue;
        }


        Coord deltaToFire = bestFireCoord - droneMotion.pos;
        Coord dirVec = normalize(deltaToFire);
        float dirToTarget = atan2(dirVec.y, dirVec.x);

        droneMotion = updateDroneMotion(
            acceleration,
            droneConfig.simTimeStep,
            droneConfig.attackSpeed,
            droneConfig.angularSpeed,
            dirToTarget,
            droneConfig.turnThreshold,
            droneMotion
        );

        Coord bombLandCoord = getBombLandCoord(droneMotion, ammoHorizontalFlightDistance);
        Coord targetAtImpact = getInterpolatedCoords(
            currentTime + ammoFlightTime, 
            droneConfig.arrayTimeStep,
            targets[bestTargetIndex],
            timeSteps
        );

        float bombMissDistance = getDistanceToTarget(bombLandCoord, targetAtImpact);
        Coord dir = { 
            .x = std::cos(droneMotion.dir), 
            .y = std::sin(droneMotion.dir) 
        };
    
        Coord aimPoint = droneMotion.pos + dir * ammoHorizontalFlightDistance;

        //push value to simArray
        simSteps[currentStep] = {
            .target = bestTargetIndex,
            .dropPoint = bestFireCoord,
            .aimPoint = aimPoint,
            .predictedTarget = targetAtImpact,
            .droneMotion = droneMotion
        };

        if (bombMissDistance < droneConfig.hitRadius) {
            LOG("Drone hit the target. Bomb miss: " << bombMissDistance);
        
            break;
        }

        if (bombLandCoord == targetAtImpact) {
            LOG("Direct hit");

            break;
        }

        prevTargetIndex = bestTargetIndex;
        prevDirToTarget = dirToTarget; 
        ++currentStep;
        currentTime += droneConfig.simTimeStep;
    }

    freeTargets(targets, targetsCount);
    
     writeSimulationJSONFile( 
        simSteps,
        currentStep
     );

     delete[] simSteps;

    return 0;
}

