#pragma once
#include "interfaces/ballistic_solver.hpp"
#include "interfaces/config_loader.hpp"
#include "interfaces/target_provider.hpp"
#include "interfaces/drone_state.hpp"


class MissionProcessor {
    const int MAX_STEPS = 10000;
    std::unique_ptr<ITargetProvider>  targets;
    std::unique_ptr<IBallisticSolver> ballisticSolver;
    DroneConfig droneConfig;
    AmmoParams ammo;
    DroneContext droneContext;
    int currentStep = 1;
    int prevTargetIndex = -1;
    float currentTime = 0.0f;
    float prevDirToTarget;
    float ammoFlightTime;
    float ammoHorizontalFlightDistance;
    bool hit = false;
    std::unique_ptr<IDroneState> state;

    public:
        MissionProcessor(std::unique_ptr<ITargetProvider> targetProvider, std::unique_ptr<IBallisticSolver> solver);

        auto init(std::unique_ptr<IConfigLoader> configLoader) -> void;
        auto hasNext() -> bool;
        auto getCurrentStep() -> int;
        auto step() -> SimulationStep;
        auto reset() -> void;
        auto changeSolver(std::unique_ptr<IBallisticSolver> ballisticSolver) -> void;
};