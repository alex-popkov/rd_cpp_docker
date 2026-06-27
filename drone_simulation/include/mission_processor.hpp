#pragma once
#include <vector>
#include "interfaces/ballistic_solver.hpp"
#include "simulation.hpp"

class MissionProcessor {
public:
  explicit MissionProcessor(std::unique_ptr<IBallisticSolver> solver);

  auto init(const dlink::AmmoCfg& ammoCfg, float altitude, float attackSpd) -> void;
  auto process(const dlink::Telemetry& telem, const std::vector<dlink::TargetPos>& targets, int targetCount) -> MissionResult;
  auto reset() -> void;
  auto changeSolver(std::unique_ptr<IBallisticSolver> ballisticSolver) -> void;

private:
  auto evaluateTarget(
    int targetIndex, const DroneTelemetry& telemetry, const Coord& targetPos, const Coord& targetVelocity, Coord& outFireCoord) -> float;
  auto checkDrop(const DroneTelemetry& telemetry, const Coord& targetPos) -> bool;
  auto computeControl(const DroneTelemetry& telemetry, const Coord& firePoint) -> dlink::Control;

  std::unique_ptr<IBallisticSolver> ballisticSolver;
  AmmoParams ammo;
  int prevTargetIndex = -1;
  float ammoFlightTime = 0.0f;
  float ammoHorizontalFlightDistance = 0.0f;
  float acceleration = 0.0f;
  float attackSpeed = 0.0f;
  float hitRadius = 0.0f;
  bool initialized = false;

  // Для обчислення швидкості цілей (чекер шле тільки позицію, не швидкість)
  std::vector<Coord> prevTargetPositions;
  std::vector<Coord> targetVelocities;
  float prevTime = -1.0f;
};
