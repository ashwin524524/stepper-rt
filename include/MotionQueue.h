#pragma once
#include <Arduino.h>
#include "Config.h"

enum class MotionType : uint8_t {
  Straight,     // value = distance_mm
  TurnInPlace,  // value = angle_deg
  Arc           // dx_mm, dy_mm
};

struct MotionCmd {
  MotionType type;
  float value;   // Straight: distance_mm, TurnInPlace: angle_deg, Arc: unused (0)
  float dx_mm;   // Arc only
  float dy_mm;   // Arc only
  float time_s;
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