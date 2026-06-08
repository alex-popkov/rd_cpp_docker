#pragma once
#include <cstring>
#include <memory>
#include "simulation.hpp"
#include "interfaces/ballistic_solver.hpp"
#include "interfaces/target_provider.hpp"
#include "interfaces/config_loader.hpp"

enum class SolverType   { ANALYTICAL };
enum class ProviderType { JSON };
enum class LoaderType   { FILE };

std::unique_ptr<IBallisticSolver> createSolver(SolverType type, const DroneConfig& config);
std::unique_ptr<ITargetProvider> createProvider(ProviderType type, const std::string path);
std::unique_ptr<IConfigLoader> createLoader(LoaderType type, const std::string c, const std::string a);