#include "solvers/analytical_solver.hpp"

AnalyticalSolver::AnalyticalSolver(const DroneConfig& config)
  : droneConfig(config)
{
}

auto AnalyticalSolver::solve(const Coord& dronePosition, const Coord& targetPosition, const AmmoParams& ammo) -> Coord
{
  float flightTime = getAmmoFlightTime(droneConfig, ammo);
  float ammoHorizontalFlightDistance = getAmmoHorizontalFlightDistance(droneConfig.attackSpeed, flightTime, ammo);
  float distanceToTarget = getDistanceToTarget(targetPosition, dronePosition);
  float ratio = getRatio(distanceToTarget, ammoHorizontalFlightDistance);

  return getFireCoords(targetPosition, dronePosition, std::max(ratio, 0.01f));
}