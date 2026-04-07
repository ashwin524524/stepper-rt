#pragma once
#include <Arduino.h>
#include <math.h>

// Time-based accel/cruise/decel profile with specified start and end velocity.
// Works in 1D "steps" domain.
// If an exact solution within limits can't be found, it will clamp gracefully.
class TrapezoidProfile {
public:
  // distanceSteps can be negative. v0/v1 can be negative (consistent direction).
  // vmaxStepsPerS and amaxStepsPerS2 are positive limits.
  void plan(float distanceSteps, float totalTimeS,
            float v0StepsPerS, float v1StepsPerS,
            float vmaxStepsPerS, float amaxStepsPerS2)
  {
    // Handle sign by flipping to positive direction
    float sign = (distanceSteps >= 0.0f) ? 1.0f : -1.0f;

    D_  = fabsf(distanceSteps);
    T_  = fmaxf(1e-4f, totalTimeS);

    v0_ = sign * v0StepsPerS;
    v1_ = sign * v1StepsPerS;

    vmax_ = fmaxf(1e-6f, vmaxStepsPerS);
    amax_ = fmaxf(1e-6f, amaxStepsPerS2);

    // Clamp start/end speeds to vmax (same direction as motion)
    v0_ = clampAbs_(v0_, vmax_);
    v1_ = clampAbs_(v1_, vmax_);

    // If distance is tiny, just interpolate with no motion
    if (D_ < 1e-6f) {
      a_ = 0.0f; vPeak_ = 0.0f; ta_=tc_=td_=0.0f;
      sA_ = sB_ = 0.0f;
      sign_ = sign;
      return;
    }

    // We search for a feasible accel (<=amax) that yields a trapezoid meeting D,T,v0,v1.
    // Use highest feasible accel for responsiveness.
    float bestA = 0.0f;
    float bestV = 0.0f;
    float bestTa=0.0f, bestTc=0.0f, bestTd=0.0f;
    bool found = false;

    auto tryAccel = [&](float a)->bool {
      // Quadratic solution for vPeak given accel a and time T:
      // v^2 - (v0+v1+aT)*v + (v0^2+v1^2)/2 + aD = 0
      float B = (v0_ + v1_ + a*T_);
      float C = 0.5f*(v0_*v0_ + v1_*v1_) + a*D_;
      float disc = B*B - 4.0f*C;
      if (disc < 0.0f) return false;

      float v = 0.5f * (B - sqrtf(disc)); // choose smaller root
      if (v < fmaxf(v0_, v1_) - 1e-4f) return false;
      if (v > vmax_ + 1e-4f) return false;

      float ta = (v - v0_) / a;
      float td = (v - v1_) / a;
      float tc = T_ - ta - td;

      if (ta < -1e-4f || td < -1e-4f || tc < -1e-4f) return false;

      // accept (tiny negative due to float)
      ta = fmaxf(0.0f, ta);
      td = fmaxf(0.0f, td);
      tc = fmaxf(0.0f, tc);

      bestA = a; bestV = v; bestTa=ta; bestTc=tc; bestTd=td;
      return true;
    };

    // First try with amax
    if (tryAccel(amax_)) {
      found = true;
    } else {
      // Binary search lower accel until feasible
      float lo = 1e-6f;
      float hi = amax_;
      bool ok = false;

      for (int i = 0; i < 40; i++) {
        float mid = 0.5f*(lo+hi);
        if (tryAccel(mid)) { ok = true; lo = mid; } // try higher accel
        else { hi = mid; }
      }
      found = ok;
    }

    if (!found) {
      // Last-resort fallback: constant-velocity that matches distance/time,
      // ignoring requested v0/v1 (keeps direction consistent).
      float vavg = D_ / T_;
      vavg = fminf(vavg, vmax_);
      a_ = 0.0f;
      v0_ = vavg;
      v1_ = vavg;
      vPeak_ = vavg;
      ta_ = 0.0f; tc_ = T_; td_ = 0.0f;
      sA_ = 0.0f; sB_ = D_;
      sign_ = sign;
      return;
    }

    // Commit chosen parameters
    a_ = bestA;
    vPeak_ = bestV;
    ta_ = bestTa; tc_ = bestTc; td_ = bestTd;

    // Precompute segment boundary positions in + direction
    // Accel segment end:
    sA_ = v0_*ta_ + 0.5f*a_*ta_*ta_;
    // Cruise segment end:
    sB_ = sA_ + vPeak_*tc_;

    // Numerical clamp to final distance
    if (sB_ > D_) sB_ = D_;
    sign_ = sign;
  }

  float duration() const { return T_; }

  // position (steps) at time t
  float pos(float t) const {
    t = clampT_(t);
    float s = 0.0f;

    if (t <= ta_) {
      s = v0_*t + 0.5f*a_*t*t;
    } else if (t <= ta_ + tc_) {
      float dt = t - ta_;
      s = sA_ + vPeak_*dt;
    } else {
      float dt = t - (ta_ + tc_);
      // decel from vPeak down to v1 with accel = -a
      s = sB_ + vPeak_*dt - 0.5f*a_*dt*dt;
    }

    // Clamp and apply sign
    if (s < 0.0f) s = 0.0f;
    if (s > D_) s = D_;
    return sign_ * s;
  }

    // velocity (steps/s) at time t
  float vel(float t) const {
    t = clampT_(t);

    float v = 0.0f;
    if (t <= ta_) {
      v = v0_ + a_ * t;
    } else if (t <= ta_ + tc_) {
      v = vPeak_;
    } else {
      float dt = t - (ta_ + tc_);
      v = vPeak_ - a_ * dt;
    }

    return sign_ * v;
  }

private:
  static float clampAbs_(float x, float lim) {
    if (x > lim) return lim;
    if (x < -lim) return -lim;
    return x;
  }

  float clampT_(float t) const {
    if (t < 0.0f) return 0.0f;
    if (t > T_) return T_;
    return t;
  }

  float sign_ = 1.0f;

  float D_ = 0.0f;   // abs distance
  float T_ = 0.0f;

  float vmax_ = 0.0f;
  float amax_ = 0.0f;

  float v0_ = 0.0f;     // start vel in + direction
  float v1_ = 0.0f;     // end vel in + direction
  float vPeak_ = 0.0f;  // peak vel in + direction
  float a_ = 0.0f;

  float ta_ = 0.0f, tc_ = 0.0f, td_ = 0.0f;

  float sA_ = 0.0f; // position at end accel (in + direction)
  float sB_ = 0.0f; // position at end cruise (in + direction)
};