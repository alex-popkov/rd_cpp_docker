#pragma once
#include <iostream>
#include "interfaces/ballistic_solver.hpp"
#include "interfaces/config_loader.hpp"
#include "interfaces/target_provider.hpp"


class MissionProcessor {
    const int MAX_STEPS = 10000;
    const float g = 9.81f;
    ITargetProvider*  targets;
    IBallisticSolver* ballisticSolver;
    DroneConfig droneConfig;
    AmmoParams ammo;
    DroneMotion droneMotion;
    int currentStep = 1;
    int prevTargetIndex = -1;
    float currentTime = 0.0f;
    float prevDirToTarget;
    float ammoFlightTime;
    float ammoHorizontalFlightDistance;
    float acceleration;
    bool hit = false;

    public:
        MissionProcessor(ITargetProvider* targetProvider, IBallisticSolver* solver): targets(targetProvider), ballisticSolver(solver) {
        }

        void init(IConfigLoader* configLoader) { 
            configLoader->load();
            droneConfig = configLoader->getConfig();
            ammo = configLoader->getAmmoParams();

            ammoFlightTime = getAmmoFlightTime(droneConfig, ammo, g);
            ammoHorizontalFlightDistance =  getAmmoHorizontalFlightDistance(
                droneConfig.attackSpeed,
                ammoFlightTime,
                ammo,
                g
            );
            acceleration = droneConfig.attackSpeed * droneConfig.attackSpeed / (2.0f * droneConfig.accelPath);
            droneMotion = {
                .speed = 0.0f,
                .dir = droneConfig.initialDir,
                .pos = {
                    .x = droneConfig.startPos.x,
                    .y = droneConfig.startPos.y
                },
                .state = STOPPED
            };
        }

        auto hasNext() -> bool { 
            return currentStep <= MAX_STEPS && !hit;
        }

        auto getCurrentStep() -> int { 
            return currentStep;
        }

        auto step() -> SimulationStep {
            int bestTargetIndex = -1;
            float bestTotalTime = INFINITY;
            Coord bestFireCoord = {
                .x = 0,
                .y = 0
            };

            //iteration by targets
            for (int i = 0; i < targets->getTargetCount(); ++i) {
                Coord target = getInterpolatedCoords(currentTime, droneConfig.arrayTimeStep, targets->getTarget(i), targets->getTimeSteps());

                // balistic
                float distanceToTarget = getDistanceToTarget(target, droneMotion.pos);
                if (distanceToTarget < 1e-6f) {
                    std::cout << "Distance to the target " << i << " is near or less then 0\n" << std::endl;
                    continue;
                }
                float ratio = getRatio(distanceToTarget, ammoHorizontalFlightDistance);

                Coord fireCoord = getFireCoords(target, droneMotion.pos, ratio);
                float distanceToFire = getDistanceToTarget(fireCoord, droneMotion.pos);
                float droneToFireTime = distanceToFire / droneConfig.attackSpeed;
                float totalTimeRough = droneToFireTime + ammoFlightTime;

                Coord predictedTarget = getPredictedTargetCoords(
                    currentTime, droneConfig.arrayTimeStep, targets->getTarget(i), target, totalTimeRough, targets->getTimeSteps()
                );

                //predicted balistic
                float newDistanceToTarget = getDistanceToTarget(predictedTarget, droneMotion.pos);
                if (newDistanceToTarget < 1e-6f) {
                    std::cout << "Predicted distance to the target " << i << " is near or less then 0\n" << std::endl;
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
                std::cout << "No valid target at this step, skipping"<< std::endl;
                ++currentStep;
                currentTime += droneConfig.simTimeStep;

                SimulationStep emptyStep = {
                    .target          = -1,
                    .dropPoint       = {0, 0},
                    .aimPoint        = {0, 0},
                    .predictedTarget = {0, 0},
                    .droneMotion     = droneMotion
                };

                return emptyStep;
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
                targets->getTarget(bestTargetIndex),
                targets->getTimeSteps()
            );

            float bombMissDistance = getDistanceToTarget(bombLandCoord, targetAtImpact);
            Coord dir = { 
                .x = std::cos(droneMotion.dir), 
                .y = std::sin(droneMotion.dir) 
            };
        
            Coord aimPoint = droneMotion.pos + dir * ammoHorizontalFlightDistance;

            SimulationStep simulationStep = {
                .target = bestTargetIndex,
                .dropPoint = bestFireCoord,
                .aimPoint = aimPoint,
                .predictedTarget = targetAtImpact,
                .droneMotion = droneMotion
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
            prevDirToTarget = dirToTarget; 


            currentStep++;
            currentTime += droneConfig.simTimeStep;

            return simulationStep;
        }

        void reset() {
            hit = false;
            prevTargetIndex = -1;
            prevDirToTarget = droneConfig.initialDir;
            currentStep = 1; 
            currentTime = 0.0f;
            droneMotion = {
                .speed = 0.0f,
                .dir = droneConfig.initialDir,
                .pos = {
                    .x = droneConfig.startPos.x,
                    .y = droneConfig.startPos.y
                },
                .state = STOPPED
            };
        }

        void changeSolver(IBallisticSolver* ballisticSolver) {
            this->ballisticSolver = ballisticSolver;
        }
};