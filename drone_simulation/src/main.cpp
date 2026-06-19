#include <iostream>
#include <cmath>
#include <cstring>
#include "json.hpp"
#include "simulation.hpp"
#include "mission_processor.hpp"
#include "config_loaders/factory.hpp"
#include "drone_states/state_stopped.hpp"
#include "providers/thread_safe_target_provider.hpp"

using json = nlohmann::json;

#define ENABLE_LOG 1
#define ENABLE_DEBUG 0

#if ENABLE_LOG
#define LOG(msg) std::cout << "[LOG] " << msg << std::endl
#else
#define LOG(msg)
#endif

#if ENABLE_DEBUG
#define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
#define DEBUG(msg)
#endif

auto main(int argc, char* argv[]) -> int
{
  try {
    const std::string configPath = (argc > 1) ? argv[1] : "config.json";
    const std::string ammoPath = (argc > 2) ? argv[2] : "ammo.json";
    const std::string targetsPath = (argc > 3) ? argv[3] : "targets.json";
    const std::string solverArg = (argc > 4) ? argv[4] : "analytical";
    const std::string ballisticTablePath = (argc > 5) ? argv[5] : "ballistic_table.txt";

    auto loader = createLoader(LoaderType::FILE, configPath, ammoPath);

    loader->load();
    DroneConfig droneConfig = loader->getConfig();
    DronePhysics physics(droneConfig);
    auto targets = std::make_unique<ThreadSafeTargetProvider>(targetsPath, droneConfig.arrayTimeStep);
    auto* targetsPtr = targets.get();

    SolverType solverType = (solverArg == "analytical") ? SolverType::ANALYTICAL : SolverType::TABLE;
    auto ballisticSolver = createSolver(solverType, droneConfig, ballisticTablePath);
    std::vector<SimulationStep> simSteps;

    MissionProcessor missionProcessor(std::move(targets), std::move(ballisticSolver), &physics);
    missionProcessor.init(std::move(loader));

    // write initial sim data
    simSteps.push_back({.hit = false,
                        .target = -1,
                        .droneDirection = droneConfig.initialDir,
                        .dropPoint = {0, 0},
                        .aimPoint = {0, 0},
                        .predictedTarget = {0, 0},
                        .dronePosition = droneConfig.startPos,
                        .droneState = StateStopped::NAME});

    // simulation loop
    int physicsStepsPerSim = static_cast<int>(droneConfig.simTimeStep / droneConfig.physicsTimeStep);
    float targetUpdateAccum = 0.0f;

    while (missionProcessor.hasNext()) {
      for (int i = 0; i < physicsStepsPerSim; i++) {
        physics.stepPhysics(droneConfig.physicsTimeStep);
      }

      targetUpdateAccum += droneConfig.simTimeStep;
      if (targetUpdateAccum >= droneConfig.arrayTimeStep) {
        targetsPtr->updateTargets();
        targetUpdateAccum -= droneConfig.arrayTimeStep;
      }

      SimulationStep simulationStep = missionProcessor.step();
      if (simulationStep.target >= 0) {
        simSteps.push_back(simulationStep);
      }
    }

    writeSimulationJSONFile(simSteps);

    return 0;
  }
  catch (const std::exception& error) {
    LOG("Error: " << error.what());

    return 1;
  }
}
