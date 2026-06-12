#include "simulation.hpp"
#include "config_loaders/factory.hpp"
#include "solvers/analytical_solver.hpp"
#include "solvers/table_solver.hpp"
#include "providers/json_target_provider.hpp"
#include "config_loaders/file_config_loader.hpp"

auto createSolver(SolverType type, const DroneConfig& config, const std::string tablePath) -> std::unique_ptr<IBallisticSolver> {
    switch (type) {
        case SolverType::ANALYTICAL: 
            return std::make_unique<AnalyticalSolver>(config);
            
        case SolverType::TABLE: 
            return std::make_unique<TableSolver>(config, tablePath);
    }
    
    return nullptr;
}

auto createProvider(ProviderType type, const std::string param) -> std::unique_ptr<ITargetProvider> {
    switch (type) {
        case ProviderType::JSON: 
            return std::make_unique<JsonTargetProvider>(param);
    }

    return nullptr;
}

auto createLoader(LoaderType type, const std::string droneConfigPath, const std::string ammoConfigPath) -> std::unique_ptr<IConfigLoader> {
    switch (type) {
        case LoaderType::FILE: 
            return std::make_unique<FileConfigLoader>(droneConfigPath, ammoConfigPath);
    }

    return nullptr;
}