#pragma once
#include "simulation.hpp"

class IBallisticSolver {
public:
  virtual Coord solve(const Coord& dronePosition, const Coord& targetPosition, const AmmoParams& ammmo) = 0;
  virtual ~IBallisticSolver() {}
};