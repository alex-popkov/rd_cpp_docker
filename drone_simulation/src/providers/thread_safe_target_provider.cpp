#include "providers/thread_safe_target_provider.hpp"

ThreadSafeTargetProvider::ThreadSafeTargetProvider(const std::string path, float arrayTimeStep)
{
  json jsonFile = parseJSONfile(path);
  this->count = jsonFile["targetCount"];
  this->arrayTimeStep = arrayTimeStep;
  this->trajectories = fillTargets(jsonFile);
  this->currentIndex.resize(this->trajectories.size(), 0);
  this->currentTargets.resize(this->trajectories.size());

  for (size_t i = 0; i < this->trajectories.size(); i++) {
    this->currentTargets[i].pos = this->trajectories[i][0];

    // Швидкість — куди і як швидко ціль рухається
    int next = 1 % this->trajectories[i].size();  // наступний вузол (перестраховка якщо всього 1 вузол в массиві позицій цілі)
    this->currentTargets[i].velocity = (this->trajectories[i][next] - this->trajectories[i][0]) / arrayTimeStep;
  }
}

ThreadSafeTargetProvider::~ThreadSafeTargetProvider() {}

auto ThreadSafeTargetProvider::getTargetCount() -> int
{
  return this->count;
}

auto ThreadSafeTargetProvider::getTarget(int i) -> Target
{
  return this->currentTargets[i];
}

auto ThreadSafeTargetProvider::updateTargets() -> void
{
  for (size_t i = 0; i < this->trajectories.size(); i++) {
    int size = this->trajectories[i].size();

    // Просунути індекс на 1, із зацикленням
    this->currentIndex[i] = (this->currentIndex[i] + 1) % size;

    int curr = this->currentIndex[i];
    int next = (curr + 1) % size;

    this->currentTargets[i].pos = this->trajectories[i][curr];
    this->currentTargets[i].velocity = (this->trajectories[i][next] - this->trajectories[i][curr]) / this->arrayTimeStep;
  }
}