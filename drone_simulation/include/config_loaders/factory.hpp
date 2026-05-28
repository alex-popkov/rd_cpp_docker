#pragma once
#include "simulation.hpp"
#include "interfaces/ballistic_solver.hpp"
#include "interfaces/target_provider.hpp"
#include "interfaces/config_loader.hpp"

enum class SolverType   { ANALYTICAL };
enum class ProviderType { JSON };
enum class LoaderType   { FILE };

IBallisticSolver* createSolver(SolverType type, const DroneConfig& config);
ITargetProvider* createProvider(ProviderType type, const char* path);
IConfigLoader* createLoader(LoaderType type, const char* c, const char* a);