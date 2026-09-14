#include "providers/json_target_provider.hpp"

JsonTargetProvider::JsonTargetProvider(const std::string path)
{
  json jsonFile = parseJSONfile(path);
  count = jsonFile["targetCount"];
  targets = fillTargets(jsonFile);
}

JsonTargetProvider::~JsonTargetProvider() {}

auto JsonTargetProvider::getTargetCount() -> int
{
  return count;
}

auto JsonTargetProvider::getTarget(int i) -> Target
{
  return {.pos = {static_cast<float>(i), static_cast<float>(i)}, .velocity = {1, 1}};
}
