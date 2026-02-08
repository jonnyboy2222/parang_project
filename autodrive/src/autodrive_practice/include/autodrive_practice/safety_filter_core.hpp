#pragma once
#include <string>

namespace autodrive_practice
{
struct SafetyResult
{
  bool stop{false};
  std::string reason;
};

class SafetyFilterCore
{
public:
  explicit SafetyFilterCore(float stop_threshold_m = 2.0f)
  : stop_threshold_m_(stop_threshold_m) {}

  void set_stop_threshold(float v) { stop_threshold_m_ = v; }
  float stop_threshold() const { return stop_threshold_m_; }

  // 입력(distance_m)을 보고 "정지해야 하는지" 판단
  SafetyResult update(float distance_m) const
  {
    SafetyResult r;
    if (distance_m < stop_threshold_m_) {
      r.stop = true;
      r.reason = "Obstacle too close";
    } else {
      r.stop = false;
      r.reason = "Safe";
    }
    return r;
  }

private:
  float stop_threshold_m_;
};
}  // namespace autodrive_practice
