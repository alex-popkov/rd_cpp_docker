#pragma once

#include "interfaces/ballistic_solver.hpp"

class AnalyticalSolver : public IBallisticSolver {
    DroneConfig droneConfig;

    public:

        AnalyticalSolver(const DroneConfig& config):  droneConfig(config) {
        }

        auto  solve(
            const Coord& dronePosition,
            const Coord& targetPosition,
            const AmmoParams& ammo
        ) -> Coord override {
            const float g = 9.81f; 
            float flightTime = getAmmoFlightTime(droneConfig, ammo, g);
            float ammoHorizontalFlightDistance  = getAmmoHorizontalFlightDistance(droneConfig.attackSpeed, flightTime, ammo, g);
            float distanceToTarget = getDistanceToTarget(targetPosition, dronePosition);
            float ratio = getRatio(distanceToTarget, ammoHorizontalFlightDistance);

            return getFireCoords(targetPosition, dronePosition, ratio);
        }
        
        
};