#include "simulation.hpp"
#include "config_loaders/factory.hpp"
#include "solvers/analytical_solver.hpp"
#include "providers/json_target_provider.hpp"
#include "config_loaders/file_config_loader.hpp"

auto createSolver(SolverType type, const DroneConfig& config) -> IBallisticSolver* {
    switch (type) {
        case SolverType::ANALYTICAL: 
            return new AnalyticalSolver(config);
    }

    return nullptr;
}

auto createProvider(ProviderType type, const std::string param) -> ITargetProvider* {
    switch (type) {
        case ProviderType::JSON: 
            return new JsonTargetProvider(param);
    }

    return nullptr;
}

auto createLoader(LoaderType type, const std::string droneConfigPath, const std::string ammoConfigPath) -> IConfigLoader* {
    switch (type) {
        case LoaderType::FILE: 
            return new FileConfigLoader(droneConfigPath, ammoConfigPath);
    }

    return nullptr;
}