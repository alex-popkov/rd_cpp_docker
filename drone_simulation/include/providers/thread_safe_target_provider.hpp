#pragma once
#include <cstring>
#include <vector>
#include "interfaces/target_provider.hpp"

class ThreadSafeTargetProvider : public ITargetProvider {
  std::vector<std::vector<Coord>> trajectories;
  // Поточний зріз стану всіх цілей
  std::vector<Target> currentTargets;
  // Для кожної цілі — на якому вузлі вона зараз
  std::vector<int> currentIndex;
  float arrayTimeStep;
  int count;
  mutable std::mutex mtx;

public:
  ThreadSafeTargetProvider(const std::string path, float arrayTimeStep);
  ~ThreadSafeTargetProvider() override;

  auto getTargetCount() -> int override;
  auto getTarget(int i) -> Target override;
  auto updateTargets() -> void;
};