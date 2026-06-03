#include "solvers/analytical_solver.hpp"


AnalyticalSolver::AnalyticalSolver(const DroneConfig& config):  droneConfig(config) {
}

auto  AnalyticalSolver::solve(
            const Coord& dronePosition,
            const Coord& targetPosition,
            const AmmoParams& ammo
) -> Coord {
    const float g = 9.81f; 
    float flightTime = getAmmoFlightTime(droneConfig, ammo, g);
    float ammoHorizontalFlightDistance  = getAmmoHorizontalFlightDistance(droneConfig.attackSpeed, flightTime, ammo, g);
    float distanceToTarget = getDistanceToTarget(targetPosition, dronePosition);
    float ratio = getRatio(distanceToTarget, ammoHorizontalFlightDistance);

    return getFireCoords(targetPosition, dronePosition, std::max(ratio, 0.01f));
}