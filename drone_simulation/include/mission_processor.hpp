#pragma once
#include "interfaces/ballistic_solver.hpp"
#include "interfaces/config_loader.hpp"
#include "interfaces/target_provider.hpp"

class MissionProcessor {
    ITargetProvider*  targets;
    IBallisticSolver* ballisticSolver;
    DroneConfig droneConfig;
    AmmoParams ammo;
    int currentIdx = 0;

    public:
        MissionProcessor(ITargetProvider* targetProvider, IBallisticSolver* solver): targets(targetProvider), ballisticSolver(solver) {
        }

        void init(IConfigLoader* configLoader) { 
            configLoader->load();
            droneConfig = configLoader->getConfig();
            ammo = configLoader->getAmmoParams();
        }

        bool hasNext() { 
            return currentIdx < targets->getTargetCount();
        }

        Coord step() {
            Coord* trajectory = targets->getTarget(currentIdx);
            Coord  targetPos  = trajectory[0]; // TODO: use currentTime?
            Coord  dronePos   = droneConfig.startPos;
            Coord  dropPoint  = ballisticSolver->solve(dronePos, targetPos, ammo);
            currentIdx++;
            
            return dropPoint;
        }

        void reset() {
            currentIdx = 0;
        }

        void changeSolver(IBallisticSolver* ballisticSolver) {
            this->ballisticSolver = ballisticSolver;
        }
};