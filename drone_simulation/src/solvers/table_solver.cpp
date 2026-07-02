#include "solvers/table_solver.hpp"


TableSolver::TableSolver(const DroneConfig& config, std::string path):  droneConfig(config) {
    this->table.load(path);
}

auto  TableSolver::solve(
            const Coord& dronePosition,
            const Coord& targetPosition,
            const AmmoParams& ammo
) -> Coord {
    Result result = this->table.lookup(this->droneConfig.altitude, this->droneConfig.attackSpeed, ammo.mass, ammo.drag, ammo.lift);
    float distanceToTarget = getDistanceToTarget(targetPosition, dronePosition);
    float ratio = getRatio(distanceToTarget, result.hDist);
    
    return getFireCoords(targetPosition, dronePosition, ratio);
}