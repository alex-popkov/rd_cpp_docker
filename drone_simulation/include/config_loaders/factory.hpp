#pragma once
#include <memory>
#include <string>

struct DroneConfig;
class IBallisticSolver;
class ITargetProvider;
class IConfigLoader;

enum class SolverType { ANALYTICAL, TABLE };
enum class ProviderType { JSON, THREAD_SAFE };
enum class LoaderType { FILE };

std::unique_ptr<IBallisticSolver> createSolver(SolverType type, const DroneConfig& config, const std::string tablePath = "");
std::unique_ptr<ITargetProvider> createProvider(ProviderType type, const std::string path, float arrayTimeStep, float timeScale);
std::unique_ptr<IConfigLoader> createLoader(LoaderType type, const std::string c, const std::string a);