#pragma once

#include <chrono>
#include <string>

#include <glog/logging.h>
#include <ros/duration.h>
#include <ros/time.h>

namespace timer_common {
class Timer {
public:
  Timer(const std::string &name) : name_(name) { Start(); }
  ~Timer() {}
  /* Start timer */
  void Start() { start_ = std::chrono::steady_clock::now(); }
  /* stop timer and return interval, in 'second' */
  double Stop() {
    end_ = std::chrono::steady_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end_ - start_);
    LOG(INFO) << name_.c_str() << ": " << duration.count() / 1000.0 << " ms";
    start_ = end_;
    return static_cast<double>(duration.count() / 1000000.0);
  }

private:
  std::string name_;
  std::chrono::time_point<std::chrono::steady_clock> start_;
  std::chrono::time_point<std::chrono::steady_clock> end_;

}; // class Timer
static double CurrentTime() {
  auto ts = std::chrono::system_clock::now();
  return static_cast<double>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          ts.time_since_epoch())
          .count() /
      1000000.0);
}

} // namespace timer_common
