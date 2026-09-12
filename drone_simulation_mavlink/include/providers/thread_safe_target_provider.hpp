#pragma once
#include <cstring>
#include <vector>
#include <atomic>
#include <mutex>
#include "interfaces/target_provider.hpp"

class ThreadSafeTargetProvider : public ITargetProvider {
public:
  ThreadSafeTargetProvider(const std::string path, float arrayTimeStep, float timeScale);
  ~ThreadSafeTargetProvider() override;

  auto getTargetCount() -> int override;
  auto getTarget(int i) -> Target override;
  auto run() -> void;
  auto start() -> void;
  auto stop() -> void;
  auto isThreadReady() const -> bool;

private:
  auto updateTargets() -> void;

  std::vector<std::vector<Coord>> trajectories;
  std::vector<Target> currentTargets;
  std::vector<int> currentIndex;
  float arrayTimeStep;
  int count;
  float timeScale;
  mutable std::mutex mtx;
  std::atomic<bool> ready{false};
  std::atomic<bool> running{false};
  std::atomic<bool> stopFlag{false};
};
