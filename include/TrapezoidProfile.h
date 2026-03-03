#pragma once
#include <Arduino.h>
#include <math.h>

// Time-based symmetric accel/cruise/decel profile from s=0 to s=distance (steps)
// Always returns a feasible profile if T >= Tmin; otherwise we clamp to Tmin.
class TrapezoidProfile {
public:
  // distanceSteps can be negative; profile preserves sign.
  // vmaxStepsPerS, amaxStepsPerS2 are positive limits.
  void plan(float distanceSteps, float totalTimeS,
            float vmaxStepsPerS, float amaxStepsPerS2)
  {
    sign_ = (distanceSteps >= 0.0f) ? 1.0f : -1.0f;
    D_ = fabsf(distanceSteps);
    T_ = (totalTimeS <= 0.0f) ? 0.001f : totalTimeS;

    vmax_ = fmaxf(1e-6f, vmaxStepsPerS);
    amax_ = fmaxf(1e-6f, amaxStepsPerS2);

    // Minimal time with limits
    const float dTri = (vmax_ * vmax_) / amax_;
    float Tmin;
    if (D_ <= dTri) {
      Tmin = 2.0f * sqrtf(D_ / amax_);
    } else {
      Tmin = 2.0f * (vmax_ / amax_) + (D_ - dTri) / vmax_;
    }

    // Clamp if requested time is too small
    if (T_ < Tmin) T_ = Tmin;

    // Try solve with a=amax for exact T:
    // D = v*T - v^2/a  (when cruise time >= 0)
    // v = (aT - sqrt((aT)^2 - 4aD))/2
    float a = amax_;
    float disc = (a * T_) * (a * T_) - 4.0f * a * D_;

    if (disc >= 0.0f) {
      float v = (a * T_ - sqrtf(disc)) * 0.5f;
      float tc = T_ - 2.0f * v / a;

      if (tc >= -1e-6f && v <= vmax_ + 1e-6f) {
        // Trapezoid (maybe tiny tc)
        v_ = fminf(v, vmax_);
        a_ = a;
        ta_ = v_ / a_;
        tc_ = fmaxf(0.0f, T_ - 2.0f * ta_);
        td_ = ta_;
        return;
      }
    }

    // If v from amax solve exceeds vmax, force v=vmax and solve for a to meet T:
    // T = D/v + v/a  => a = v / (T - D/v)
    {
      float v = vmax_;
      float denom = (T_ - (D_ / v));
      if (denom > 1e-6f) {
        float a = v / denom;
        if (a <= amax_ + 1e-6f) {
          v_ = v;
          a_ = fmaxf(1e-6f, a);
          ta_ = v_ / a_;
          tc_ = fmaxf(0.0f, T_ - 2.0f * ta_);
          td_ = ta_;
          return;
        }
      }
    }

    // Otherwise fall back to triangular that matches T exactly (if within limits),
    // a_req = 4D/T^2, v_req = 2D/T
    {
      float a_req = 4.0f * D_ / (T_ * T_);
      float v_req = 2.0f * D_ / T_;
      a_ = fminf(a_req, amax_);
      v_ = fminf(v_req, vmax_);
      // Recompute consistent times for triangular:
      ta_ = (v_ / a_);
      tc_ = 0.0f;
      td_ = ta_;
      // T_ becomes 2*ta; if that differs from desired, we accept this safe fallback.
      T_ = 2.0f * ta_;
    }
  }

  float duration() const { return T_; }

  // position in steps at time t (0..T)
  float pos(float t) const {
    t = clampT(t);
    float s = 0.0f;

    if (t <= ta_) {
      // accel: s = 0.5*a*t^2
      s = 0.5f * a_ * t * t;
    } else if (t <= ta_ + tc_) {
      // cruise: s = s_acc + v*(t-ta)
      float s_acc = 0.5f * a_ * ta_ * ta_;
      s = s_acc + v_ * (t - ta_);
    } else {
      // decel: symmetric
      float t2 = t - (ta_ + tc_);
      float s_acc = 0.5f * a_ * ta_ * ta_;
      float s_cruise = v_ * tc_;
      // during decel: s = s_acc + s_cruise + v*t2 - 0.5*a*t2^2
      s = s_acc + s_cruise + v_ * t2 - 0.5f * a_ * t2 * t2;
    }

    // Clamp numeric noise and apply sign
    if (s > D_) s = D_;
    return sign_ * s;
  }

private:
  float clampT(float t) const {
    if (t < 0.0f) return 0.0f;
    if (t > T_) return T_;
    return t;
  }

  float sign_ = 1.0f;
  float D_ = 0.0f;     // abs distance (steps)
  float T_ = 0.0f;

  float vmax_ = 0.0f;
  float amax_ = 0.0f;

  // planned parameters
  float v_ = 0.0f;
  float a_ = 0.0f;
  float ta_ = 0.0f;  // accel time
  float tc_ = 0.0f;  // cruise time
  float td_ = 0.0f;  // decel time (equals ta)
};
