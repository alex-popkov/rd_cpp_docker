#include <iostream>
#include <fstream>
#include <cmath>
#include <cstring>
#include "json.hpp"
#include "simulation.hpp"
#include "mission_processor.hpp"
#include "config_loaders/factory.hpp"
#include "providers/thread_safe_target_provider.hpp"
#include "api/api_client.hpp"
#include "api/network.hpp"

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
    const std::string testID = (argc > 1) ? argv[1] : "T01";
    const std::string configPath = (argc > 2) ? argv[2] : "config.json";
    const std::string ammoPath = (argc > 3) ? argv[3] : "ammo.json";
    const std::string targetsPath = (argc > 4) ? argv[4] : "targets.json";
    const std::string solverArg = (argc > 5) ? argv[5] : "analytical";
    const std::string ballisticTablePath = (argc > 6) ? argv[6] : "ballistic_table.txt";

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

    auto steps = missionProcessor.getSteps();

    writeSimulationJSONFile(steps);

    auto apiClient = ApiClient("cppmiltech.com.ua", 80, 2, 2);

    std::string studentID = "2073";

    std::ifstream f("simulation.json");
    json simulation;
    f >> simulation;

    json payload;
    payload["studentId"] = studentID;
    payload["testId"] = testID;
    payload["simulation"] = simulation;
    std::string body = payload.dump();
    httplib::Headers headers = {{"x-api-key", "dz12-vX7mK4qT9r2w"}};

    bool ok = false;
    int attempt = 0;

    for (attempt = 1; attempt <= 5; ++attempt) {
      auto response = apiClient.post("/api/dz12/results", body, "application/json", headers);
      ApiCallOutcome apiCallOutcome = classifyApiCallResult(response);

      if (apiCallOutcome == ApiCallOutcome::Success) {
        ok = true;
        break;
      }

      if (apiCallOutcome == ApiCallOutcome::DoNotRetry) {
        break;
      }

      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    bool resultWrote = false;
    if (ok) {
      auto checkResponseResult = apiClient.get("/api/dz12/results/" + testID + "/" + studentID, headers);
      resultWrote = checkResponseResult && checkResponseResult->status == 200;
    }

    std::cout << "Test: " << testID << "; State: " << (resultWrote ? "Ok" : "Failed") << "; Attempts: " << attempt << std::endl;

    return 0;
  }
  catch (const std::exception& error) {
    LOG("Error: " << error.what());

    return 1;
  }
}
