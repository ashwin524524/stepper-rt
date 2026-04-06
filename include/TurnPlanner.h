#pragma once
#include <Arduino.h>
#include "TrapezoidProfile.h"
#include "Config.h"

class TurnPlanner {
public:
  void plan(float angleDeg, float totalTimeS) {
    // Convert robot linear wheel limits into angular robot limits
    // omega = 2*v / WHEELBASE
    const float omegaMaxDegS =
        (2.0f * MAX_VEL_MM_S / WHEELBASE_MM) * 180.0f / PI;

    const float alphaMaxDegS2 =
        (2.0f * MAX_ACCEL_MM_S2 / WHEELBASE_MM) * 180.0f / PI;

    profile_.plan(angleDeg, totalTimeS, 0.0f, 0.0f, omegaMaxDegS, alphaMaxDegS2);
  }

  float pos(float t) const {
    return profile_.pos(t);
  }

  float duration() const {
    return profile_.duration();
  }

private:
  TrapezoidProfile profile_;
};