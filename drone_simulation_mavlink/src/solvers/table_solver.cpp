#include "solvers/table_solver.hpp"

TableSolver::TableSolver(const DroneConfig& config, std::string path)
  : droneConfig(config)
{
  this->table.load(path);
}

auto TableSolver::solve(const AmmoParams& ammo) -> BallisticResult
{
  Result result = this->table.lookup(this->droneConfig.altitude, this->droneConfig.attackSpeed, ammo.mass, ammo.drag, ammo.lift);

  return {.flightTime = result.t, .hDist = result.hDist};
}