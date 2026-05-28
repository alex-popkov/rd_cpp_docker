#pragma once

#include "interfaces/ballistic_solver.hpp"

class AnalyticalSolver : public IBallisticSolver {
    DroneConfig droneConfig;

    public:
        AnalyticalSolver(const DroneConfig& config);

        auto  solve(
            const Coord& dronePosition,
            const Coord& targetPosition,
            const AmmoParams& ammo
        ) -> Coord override;      
};