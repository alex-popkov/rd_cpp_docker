#pragma once
#include "interfaces/ballistic_solver.hpp"
#include "interfaces/config_loader.hpp"
#include "interfaces/target_provider.hpp"


class MissionProcessor {
    const int MAX_STEPS = 10000;
    ITargetProvider*  targets;
    IBallisticSolver* ballisticSolver;
    DroneConfig droneConfig;
    AmmoParams ammo;
    DroneMotion droneMotion;
    int currentStep = 1;
    int prevTargetIndex = -1;
    float currentTime = 0.0f;
    float prevDirToTarget;
    float ammoFlightTime;
    float ammoHorizontalFlightDistance;
    float acceleration;
    bool hit = false;

    public:
        MissionProcessor(ITargetProvider* targetProvider, IBallisticSolver* solver);

        auto init(IConfigLoader* configLoader) -> void;
        auto hasNext() -> bool;
        auto getCurrentStep() -> int;
        auto step() -> SimulationStep;
        auto reset() -> void;
        auto changeSolver(IBallisticSolver* ballisticSolver) -> void;
};