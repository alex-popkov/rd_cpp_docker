#pragma once
#include <mutex>
#include <queue>

template <typename T>
class ThreadSafeQueue {
  std::queue<T> queue;
  mutable std::mutex mtx;

public:
  auto push(const T& item) -> void
  {
    std::lock_guard<std::mutex> lock(mtx);
    queue.push(item);
  }

  auto tryPop(T& out) -> bool
  {
    std::lock_guard<std::mutex> lock(mtx);
    if (queue.empty()) {
      return false;
    }
    out = queue.front();
    queue.pop();

    return true;
  }
};