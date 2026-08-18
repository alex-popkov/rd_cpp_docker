#pragma once
#include <vector>
#include "interfaces/ballistic_solver.hpp"
#include "simulation.hpp"

class MissionProcessor {
public:
  MissionProcessor(std::unique_ptr<IBallisticSolver> solver, const DroneConfig& config);

  auto init(const dlink::AmmoCfg& ammoCfg, float altitude) -> void;
  auto process(const dlink::Telemetry& telem, const std::vector<dlink::TargetPos>& targets, int targetCount) -> MissionResult;
  auto reset() -> void;
  auto changeSolver(std::unique_ptr<IBallisticSolver> ballisticSolver) -> void;

private:
  auto evaluateTarget(int targetIndex,
                      const DroneTelemetry& telemetry,
                      const Coord& targetPos,
                      const Coord& targetVelocity,
                      Coord& outFireCoord,
                      Coord& outAimCoord) -> float;
  auto checkDrop(const DroneTelemetry& telemetry, const Coord& targetPos, const Coord& targetVelocity) -> bool;

  std::unique_ptr<IBallisticSolver> ballisticSolver;
  AmmoParams ammo;
  DroneConfig droneConfig;
  int prevTargetIndex = -1;
  float ammoFlightTime = 0.0f;
  float ammoHorizontalFlightDistance = 0.0f;
  float hitRadius = 0.0f;
  bool initialized = false;
  std::vector<Coord> prevTargetPositions;
  std::vector<Coord> targetVelocities;
  float prevTime = -1.0f;
};
