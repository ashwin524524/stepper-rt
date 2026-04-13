#pragma once
#include <Arduino.h>
#include "Config.h"

enum class MotionType : uint8_t { Straight, TurnInPlace, ArcRadiusAngle };

struct MotionCmd {
  MotionType type;

  float value = 0.0f;

  // ArcRadiusAngle only
  float radius_mm = 0.0f;
  float angle_deg = 0.0f;
  // Geometry:
  //float value;   // Straight: distance_mm, TurnInPlace: angle_deg, Arc: unused

  // Timing:
  float time_s;
  // Boundary velocities (mm/s)
  // Straight: center velocity = wheel velocity
  // Arc: center velocity along arc
  // TurnInPlace: interpret as wheel surface speed magnitude (optional; usually 0)
  float v0_mm_s;
  float v1_mm_s;
};

class MotionQueue {
public:
  bool push(const MotionCmd& c) {
    if (count_ >= MOTION_QUEUE_CAP) return false;
    buf_[tail_] = c;
    tail_ = (tail_ + 1) % MOTION_QUEUE_CAP;
    count_++;
    return true;
  }

  bool pop(MotionCmd& out) {
    if (count_ == 0) return false;
    out = buf_[head_];
    head_ = (head_ + 1) % MOTION_QUEUE_CAP;
    count_--;
    return true;
  }

  void clear() { head_ = tail_ = count_ = 0; }
  size_t size() const { return count_; }

private:
  MotionCmd buf_[MOTION_QUEUE_CAP];
  size_t head_ = 0, tail_ = 0, count_ = 0;
};