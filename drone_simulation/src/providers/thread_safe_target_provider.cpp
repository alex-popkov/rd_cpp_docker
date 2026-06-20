#include <thread>
#include "providers/thread_safe_target_provider.hpp"

ThreadSafeTargetProvider::ThreadSafeTargetProvider(const std::string path, float arrayTimeStep, float timeScale)
  : arrayTimeStep(arrayTimeStep)
  , timeScale(timeScale)
{
  json jsonFile = parseJSONfile(path);
  this->count = jsonFile["targetCount"];
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
  std::lock_guard<std::mutex> lock(mtx);

  return this->count;
}

auto ThreadSafeTargetProvider::getTarget(int i) -> Target
{
  std::lock_guard<std::mutex> lock(mtx);

  return this->currentTargets[i];
}

auto ThreadSafeTargetProvider::updateTargets() -> void
{
  for (size_t i = 0; i < this->trajectories.size(); i++) {
    int size = this->trajectories[i].size();

    // +1 index, із зацикленням
    this->currentIndex[i] = (this->currentIndex[i] + 1) % size;

    int curr = this->currentIndex[i];
    int next = (curr + 1) % size;

    this->currentTargets[i].pos = this->trajectories[i][curr];
    this->currentTargets[i].velocity = (this->trajectories[i][next] - this->trajectories[i][curr]) / this->arrayTimeStep;
  }
}

auto ThreadSafeTargetProvider::run() -> void
{
  this->ready.store(true);

  while (!this->running.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  while (!this->stopFlag.load()) {
    {
      std::lock_guard<std::mutex> lock(mtx);
      this->updateTargets();
    }

    std::this_thread::sleep_for(std::chrono::duration<float>(this->arrayTimeStep / this->timeScale));
  }
}

auto ThreadSafeTargetProvider::start() -> void
{
  this->running.store(true);
}

auto ThreadSafeTargetProvider::stop() -> void
{
  this->stopFlag.store(true);
}

auto ThreadSafeTargetProvider::isThreadReady() const -> bool
{
  return this->ready.load();
}