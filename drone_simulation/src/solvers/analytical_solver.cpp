#include "solvers/analytical_solver.hpp"

AnalyticalSolver::AnalyticalSolver(const DroneConfig& config)
  : droneConfig(config)
{
}

auto AnalyticalSolver::solve(const AmmoParams& ammo) -> BallisticResult
{
  float flightTime = getAmmoFlightTime(droneConfig, ammo);
  float hDist = getAmmoHorizontalFlightDistance(droneConfig.attackSpeed, flightTime, ammo);

  return {.flightTime = flightTime, .hDist = hDist};
}