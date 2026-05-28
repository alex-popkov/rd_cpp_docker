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
    bool failed = false;
    IConfigLoader* loader = nullptr;
    ITargetProvider* targets = nullptr;
    IBallisticSolver* ballisticSolver = nullptr;
    SimulationStep* simSteps = nullptr;

    const char* configPath  = (argc > 1) ? argv[1] : "config.json";
    const char* ammoPath    = (argc > 2) ? argv[2] : "ammo.json";
    const char* targetsPath = (argc > 3) ? argv[3] : "targets.json";

    try {
        const int MAX_STEPS = 10000;
        
        loader  = createLoader(LoaderType::FILE, configPath, ammoPath);
        targets = createProvider(ProviderType::JSON, targetsPath);
        
        loader->load();
        DroneConfig droneConfig = loader->getConfig();
        
        ballisticSolver = createSolver(SolverType::ANALYTICAL, droneConfig);
        
        MissionProcessor missionProcessor(targets, ballisticSolver);
        missionProcessor.init(loader);
        
        simSteps = new SimulationStep[MAX_STEPS + 1];
        //write initial sim data
        simSteps[0] = {
            .target          = -1,
            .dropPoint       = {0, 0},
            .aimPoint        = {0, 0},
            .predictedTarget = {0, 0},
            .droneMotion = {
                .dir   = droneConfig.initialDir,
                .pos   = droneConfig.startPos,
                .state = STOPPED
            }
        };

        // simulation loop
        while (missionProcessor.hasNext()) {
            SimulationStep simulationStep = missionProcessor.step();  
            if (simulationStep.target >= 0) {
                simSteps[missionProcessor.getCurrentStep() - 1] = simulationStep;
            }
        }
        
        writeSimulationJSONFile( 
            simSteps,
            missionProcessor.getCurrentStep()
        );

    } catch (const std::exception& error) {
        LOG("Error: " << error.what());
        failed = true;
    }

    delete loader;
    delete targets;
    delete ballisticSolver;
    delete[] simSteps;

    return failed ? 1 : 0;
}

