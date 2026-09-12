#pragma once
#include "simulation.hpp"

struct BallisticResult {
  float flightTime;
  float hDist;
};

class IBallisticSolver {
public:
  virtual BallisticResult solve(const AmmoParams& ammo) = 0;
  virtual ~IBallisticSolver() {}
};