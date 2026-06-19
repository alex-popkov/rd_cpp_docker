#pragma once
#include "interfaces/ballistic_solver.hpp"
#include "interfaces/config_loader.hpp"
#include "interfaces/target_provider.hpp"
#include "drone_physics.hpp"

class MissionProcessor {
  const int MAX_STEPS = 10000;
  std::unique_ptr<ITargetProvider> targets;
  std::unique_ptr<IBallisticSolver> ballisticSolver;
  DroneConfig droneConfig;
  AmmoParams ammo;
  int currentStep = 1;
  int prevTargetIndex = -1;
  float currentTime = 0.0f;
  float ammoFlightTime;
  float ammoHorizontalFlightDistance;
  bool hit = false;
  DronePhysics* dronePhysics;

public:
  MissionProcessor(std::unique_ptr<ITargetProvider> targetProvider, std::unique_ptr<IBallisticSolver> solver, DronePhysics* physics);

  auto init(std::unique_ptr<IConfigLoader> configLoader) -> void;
  auto hasNext() -> bool;
  auto getCurrentStep() -> int;
  auto step() -> SimulationStep;
  auto reset() -> void;
  auto changeSolver(std::unique_ptr<IBallisticSolver> ballisticSolver) -> void;

private:
  auto evaluateTarget(int targetIndex, const DroneContext& droneContext, Coord& outFireCoord) -> float;
  auto getSimulationStep(int targetIndex, const Coord& fireCoord, const DroneContext& droneContext) -> SimulationStep;
};
