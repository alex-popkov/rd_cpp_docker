#include <iostream>
#include <cmath>
#include <cstring>
#include "json.hpp"
#include "simulation.hpp"
#include "mission_processor.hpp"
#include "config_loaders/factory.hpp"

using json = nlohmann::json;

#define ENABLE_LOG	1
#define ENABLE_DEBUG  0
 
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


 auto main(int argc, char* argv[]) -> int {
    std::unique_ptr<IConfigLoader> loader;
    std::unique_ptr<ITargetProvider> targets;
    std::unique_ptr<IBallisticSolver> ballisticSolver;
    std::vector<SimulationStep> simSteps;

    const std::string configPath = (argc > 1) ? argv[1] : "config.json";
    const std::string ammoPath = (argc > 2) ? argv[2] : "ammo.json";
    const std::string targetsPath = (argc > 3) ? argv[3] : "targets.json";
    const std::string ballisticTablePath = (argc > 4) ? argv[4] : "ballistic_table.txt";

    try {
        loader  = createLoader(LoaderType::FILE, configPath, ammoPath);
        targets = createProvider(ProviderType::JSON, targetsPath);
        
        loader->load();
        DroneConfig droneConfig = loader->getConfig();
        
        // ballisticSolver = createSolver(SolverType::ANALYTICAL, droneConfig);
        ballisticSolver = createSolver(SolverType::TABLE, droneConfig, ballisticTablePath);
        
        MissionProcessor missionProcessor(std::move(targets), std::move(ballisticSolver));
        missionProcessor.init(std::move(loader));
        
        //write initial sim data
        simSteps.push_back({
            .target = -1,
            .dropPoint = {0, 0},
            .aimPoint = {0, 0},
            .predictedTarget = {0, 0},
            .droneMotion = {
                .dir = droneConfig.initialDir,
                .pos = droneConfig.startPos,
                .state = STOPPED
            }
        });

        // simulation loop
        while (missionProcessor.hasNext()) {
            SimulationStep simulationStep = missionProcessor.step();  
            if (simulationStep.target >= 0) {
                simSteps.push_back(simulationStep);
            }
        }
        
        writeSimulationJSONFile(simSteps);

        return 0;
    } catch (const std::exception& error) {
        LOG("Error: " << error.what());

        return 1;
    }
}

