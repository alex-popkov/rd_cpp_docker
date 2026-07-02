#include <iostream>
#include <cmath>
#include <cstring>
#include "json.hpp"
#include "simulation.hpp"
#include "mission_processor.hpp"
#include "config_loaders/factory.hpp"
#include "providers/thread_safe_target_provider.hpp"

using json = nlohmann::json;

#define ENABLE_LOG 0
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
    auto physics = std::make_unique<DronePhysics>(droneConfig);
    auto targets = std::make_unique<ThreadSafeTargetProvider>(targetsPath, droneConfig.arrayTimeStep, droneConfig.timeScale);
    auto* targetsPtr = targets.get();
    auto* physicsPtr = physics.get();

    SolverType solverType = (solverArg == "analytical") ? SolverType::ANALYTICAL : SolverType::TABLE;
    auto ballisticSolver = createSolver(solverType, droneConfig, ballisticTablePath);

    if (ballisticSolver == nullptr) {
      LOG("Error: " << "Unknown ballistic solver");

      return 1;
    }

    MissionProcessor missionProcessor(std::move(targets), std::move(ballisticSolver), physicsPtr);
    missionProcessor.init(std::move(loader));

    std::thread providerThread(&ThreadSafeTargetProvider::run, targetsPtr);
    std::thread physicsThread(&DronePhysics::run, physicsPtr);
    std::thread missionThread(&MissionProcessor::run, &missionProcessor);

    while (!targetsPtr->isThreadReady() || !physicsPtr->isThreadReady() || !missionProcessor.isThreadReady()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    targetsPtr->start();
    physicsPtr->start();
    missionProcessor.start();

    missionThread.join();

    physicsPtr->stop();
    targetsPtr->stop();

    physicsThread.join();
    providerThread.join();

    writeSimulationJSONFile(missionProcessor.getSteps());

    return 0;
  }
  catch (const std::exception& error) {
    LOG("Error: " << error.what());

    return 1;
  }
}
