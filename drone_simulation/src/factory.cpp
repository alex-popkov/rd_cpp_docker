#include "simulation.hpp"
#include "factory.hpp"
#include "analytical_solver.hpp"
#include "json_target_provider.hpp"
#include "file_config_loader.hpp"

auto createSolver(SolverType type, const DroneConfig& config) -> IBallisticSolver* {
    switch (type) {
        case SolverType::ANALYTICAL: 
            return new AnalyticalSolver(config);
    }

    return nullptr;
}

auto createProvider(ProviderType type, const char* param) -> ITargetProvider* {
    switch (type) {
        case ProviderType::JSON: 
            return new JsonTargetProvider(param);
    }

    return nullptr;
}

auto createLoader(LoaderType type, const char* droneConfigPath, const char* ammoConfigPath) -> IConfigLoader* {
    switch (type) {
        case LoaderType::FILE: 
            return new FileConfigLoader(droneConfigPath, ammoConfigPath);
    }

    return nullptr;
}