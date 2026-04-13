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
  float sign = (distanceSteps >= 0.0f) ? 1.0f : -1.0f;

  D_  = fabsf(distanceSteps);
  T_  = fmaxf(1e-4f, totalTimeS);

  vmax_ = fmaxf(1e-6f, vmaxStepsPerS);
  amax_ = fmaxf(1e-6f, amaxStepsPerS2);

  // Work in positive-motion coordinates
  v0_ = sign * v0StepsPerS;
  v1_ = sign * v1StepsPerS;

  // Clamp endpoint speeds
  if (v0_ < 0.0f) v0_ = 0.0f;
  if (v1_ < 0.0f) v1_ = 0.0f;
  if (v0_ > vmax_) v0_ = vmax_;
  if (v1_ > vmax_) v1_ = vmax_;

  sign_ = sign;

  // Reset defaults
  shape_ = Shape::Constant;
  a_ = 0.0f;
  vPeak_ = 0.0f;
  ta_ = tc_ = td_ = 0.0f;
  t1_ = t2_ = t3_ = 0.0f;
  s1_ = s2_ = 0.0f;

  if (D_ < 1e-6f) {
    return;
  }

  // Distance of linear interpolation from v0 to v1 over full T
  const float D_linear = 0.5f * (v0_ + v1_) * T_;

  // --------------------------------------------------
  // LOW-SIDE SOLUTIONS
  // --------------------------------------------------
  // If requested distance is LESS than the linear-interp distance,
  // we need either:
  //   coast at v0, then accel to v1
  // or
  //   decel from v0 to v1, then coast at v1
  //
  // These are the cases your old solver missed.
  // --------------------------------------------------
  if (D_ < D_linear - 1e-6f) {
    if (v1_ > v0_ + 1e-6f) {
      // coast at v0, then accel to v1
      // D = v0*T + 0.5*(v1-v0)*ta
      float ta = 2.0f * (D_ - v0_ * T_) / (v1_ - v0_);

      if (ta >= 0.0f && ta <= T_) {
        float a = (v1_ - v0_) / fmaxf(ta, 1e-6f);

        if (a <= amax_ + 1e-6f) {
          shape_ = Shape::LowCoastAccel;
          a_ = a;
          ta_ = ta;
          tc_ = T_ - ta_;   // coast first
          td_ = 0.0f;

          t1_ = tc_;
          t2_ = T_;
          t3_ = T_;

          s1_ = v0_ * tc_;
          s2_ = D_;
          return;
        }
      }
    }
    else if (v0_ > v1_ + 1e-6f) {
      // decel from v0 to v1, then coast at v1
      // D = v1*T + 0.5*(v0-v1)*td
      float td = 2.0f * (D_ - v1_ * T_) / (v0_ - v1_);

      if (td >= 0.0f && td <= T_) {
        float a = (v0_ - v1_) / fmaxf(td, 1e-6f);

        if (a <= amax_ + 1e-6f) {
          shape_ = Shape::LowDecelCoast;
          a_ = a;
          ta_ = 0.0f;
          tc_ = T_ - td;   // coast last
          td_ = td;

          t1_ = td_;
          t2_ = T_;
          t3_ = T_;

          s1_ = v0_ * td_ - 0.5f * a_ * td_ * td_;
          s2_ = D_;
          return;
        }
      }
    }

    // If low-side exact solution failed, fall through to safe fallback below.
  }

  // --------------------------------------------------
  // HIGH-SIDE SOLUTION
  // accel -> cruise -> decel, with peak >= max(v0,v1)
  // --------------------------------------------------
  {
    float bestA = 0.0f;
    float bestV = 0.0f;
    float bestTa = 0.0f, bestTc = 0.0f, bestTd = 0.0f;
    bool found = false;

    auto tryAccel = [&](float a) -> bool {
      // Solve:
      // D = (v^2 - v0^2)/(2a) + v*tc + (v^2 - v1^2)/(2a)
      // T = (v-v0)/a + tc + (v-v1)/a
      //
      // Eliminating tc gives:
      // v^2 - (v0+v1+aT)v + (v0^2+v1^2)/2 + aD = 0
      float B = (v0_ + v1_ + a * T_);
      float C = 0.5f * (v0_ * v0_ + v1_ * v1_) + a * D_;
      float disc = B * B - 4.0f * C;
      if (disc < 0.0f) return false;

      float sqrtDisc = sqrtf(disc);

      // Two roots; try the larger one first since high-side wants peak above endpoints
      float cand[2] = {
        0.5f * (B + sqrtDisc),
        0.5f * (B - sqrtDisc)
      };

      for (int i = 0; i < 2; i++) {
        float v = cand[i];
        if (v < fmaxf(v0_, v1_) - 1e-5f) continue;
        if (v > vmax_ + 1e-5f) continue;

        float ta = (v - v0_) / a;
        float td = (v - v1_) / a;
        float tc = T_ - ta - td;

        if (ta < -1e-5f || td < -1e-5f || tc < -1e-5f) continue;

        bestA = a;
        bestV = v;
        bestTa = fmaxf(0.0f, ta);
        bestTc = fmaxf(0.0f, tc);
        bestTd = fmaxf(0.0f, td);
        return true;
      }

      return false;
    };

    if (tryAccel(amax_)) {
      found = true;
    } else {
      float lo = 1e-6f;
      float hi = amax_;
      bool ok = false;

      for (int i = 0; i < 40; i++) {
        float mid = 0.5f * (lo + hi);
        if (tryAccel(mid)) {
          ok = true;
          lo = mid;
        } else {
          hi = mid;
        }
      }
      found = ok;
    }

    if (found) {
      shape_ = Shape::HighTrap;
      a_ = bestA;
      vPeak_ = bestV;
      ta_ = bestTa;
      tc_ = bestTc;
      td_ = bestTd;

      t1_ = ta_;
      t2_ = ta_ + tc_;
      t3_ = T_;

      s1_ = v0_ * ta_ + 0.5f * a_ * ta_ * ta_;
      s2_ = s1_ + vPeak_ * tc_;
      return;
    }
  }

  // --------------------------------------------------
  // FINAL SAFE FALLBACK
  // --------------------------------------------------
  shape_ = Shape::Constant;
  a_ = 0.0f;
  vPeak_ = D_ / T_;
  if (vPeak_ > vmax_) vPeak_ = vmax_;
  v0_ = vPeak_;
  v1_ = vPeak_;
  ta_ = 0.0f;
  tc_ = T_;
  td_ = 0.0f;

  t1_ = 0.0f;
  t2_ = T_;
  t3_ = T_;
  s1_ = 0.0f;
  s2_ = D_;
}

  float duration() const { return T_; }

  // position (steps) at time t
  float pos(float t) const {
  t = clampT_(t);
  float s = 0.0f;

  switch (shape_) {
    case Shape::HighTrap:
      if (t <= t1_) {
        s = v0_ * t + 0.5f * a_ * t * t;
      } else if (t <= t2_) {
        float dt = t - t1_;
        s = s1_ + vPeak_ * dt;
      } else {
        float dt = t - t2_;
        s = s2_ + vPeak_ * dt - 0.5f * a_ * dt * dt;
      }
      break;

    case Shape::LowCoastAccel:
      if (t <= t1_) {
        s = v0_ * t;
      } else {
        float dt = t - t1_;
        s = s1_ + v0_ * dt + 0.5f * a_ * dt * dt;
      }
      break;

    case Shape::LowDecelCoast:
      if (t <= t1_) {
        s = v0_ * t - 0.5f * a_ * t * t;
      } else {
        float dt = t - t1_;
        s = s1_ + v1_ * dt;
      }
      break;

    case Shape::Constant:
    default:
      s = v0_ * t;
      break;
  }

  if (s < 0.0f) s = 0.0f;
  if (s > D_) s = D_;
  return sign_ * s;
}

    // velocity (steps/s) at time t
  float vel(float t) const {
  t = clampT_(t);
  float v = 0.0f;

  switch (shape_) {
    case Shape::HighTrap:
      if (t <= t1_) {
        v = v0_ + a_ * t;
      } else if (t <= t2_) {
        v = vPeak_;
      } else {
        float dt = t - t2_;
        v = vPeak_ - a_ * dt;
      }
      break;

    case Shape::LowCoastAccel:
      if (t <= t1_) {
        v = v0_;
      } else {
        float dt = t - t1_;
        v = v0_ + a_ * dt;
      }
      break;

    case Shape::LowDecelCoast:
      if (t <= t1_) {
        v = v0_ - a_ * t;
      } else {
        v = v1_;
      }
      break;

    case Shape::Constant:
    default:
      v = v0_;
      break;
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

  enum class Shape : uint8_t {
    HighTrap,        // accel -> cruise -> decel
    LowCoastAccel,   // coast at v0 -> accel to v1
    LowDecelCoast,   // decel from v0 -> coast at v1
    Constant
  };

  Shape shape_ = Shape::Constant;

  // For generic phase boundaries
  float t1_ = 0.0f, t2_ = 0.0f, t3_ = 0.0f;
  float s1_ = 0.0f, s2_ = 0.0f;

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