#pragma once
#include <Arduino.h>
#include "TrapezoidProfile.h"

class ProgressProfile {
public:
  void plan(float totalTimeS) {
    // Unit-distance progress profile: u(t) goes from 0 to 1
    // Choose vmax/amax large enough that time dominates the shape.
    // Since distance = 1.0, these are in "fraction/s" and "fraction/s^2".
    constexpr float U_MAX = 1000.0f;
    constexpr float A_MAX = 1000.0f;

    prof_.plan(1.0f, totalTimeS, 0.0f, 0.0f, U_MAX, A_MAX);
  }

  float pos(float t) const {
    float u = prof_.pos(t);
    if (u < 0.0f) u = 0.0f;
    if (u > 1.0f) u = 1.0f;
    return u;
  }

  float vel(float t) const {
    return prof_.vel(t);
  }

  float duration() const {
    return prof_.duration();
  }

private:
  TrapezoidProfile prof_;
};