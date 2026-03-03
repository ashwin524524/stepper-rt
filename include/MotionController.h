#pragma once
#include <Arduino.h>
#include "Config.h"
#include "MotionQueue.h"
#include "StepperDriver.h"
#include "TrapezoidProfile.h"
#include "GyroBNO055.h"

// Runs queued motion commands by planning left/right step profiles.
// Emits steps by sampling profile position and stepping until actual step index matches.
class MotionController {
public:
    MotionController(StepperDriver& left, StepperDriver& right, MotionQueue& q, GyroBNO055* gyro = nullptr)
        : L_(left), R_(right), q_(q), gyro_(gyro) {}


  void begin() {
    maxVelSteps_  = mmToSteps(MAX_VEL_MM_S);
    maxAccelSteps_= mmToSteps(MAX_ACCEL_MM_S2);
  }

  void update() {
    if (!active_) {
      if (!startNext_()) return;
    }

    const uint32_t nowUs = micros();
    const float t = (nowUs - segStartUs_) * 1e-6f;

    // Desired positions from profile (in steps, can be negative)
    const float lPos = lProf_.pos(t);
    const float rPos = rProf_.pos(t);

    float lCmd = lPos;
    float rCmd = rPos;

    // Gyro P correction ONLY during Turn segments
    /*
    if (segIsTurn_ && gyro_) {
      float h = gyro_->headingDeg(); // [-180,180)

    Serial.print("ANGLE: "); Serial.println(h);

      float err = wrapDeg_(turnTargetHeadingDeg_ - h);

      float corr = TURN_KP_STEPS_PER_DEG * err;
      if (corr > TURN_CORR_MAX_STEPS) corr = TURN_CORR_MAX_STEPS;
      if (corr < -TURN_CORR_MAX_STEPS) corr = -TURN_CORR_MAX_STEPS;

      // If we need more CCW (err positive), increase differential:
      // left more negative, right more positive
      lCmd = lCmd - corr;
      rCmd = rCmd + corr;
    }
    */
    

    runAxisTo_(L_, lCmd, lSentSteps_);
    runAxisTo_(R_, rCmd, rSentSteps_);


    // Step until our integer "sent step index" catches up to desired position.
    // This is robust and does not require timers; just call update() fast.
    //runAxisTo_(L_, lPos, lSentSteps_);
    //runAxisTo_(R_, rPos, rSentSteps_);

    if (t >= segDurationS_) {
      // Ensure we end exactly on target step count
      runAxisTo_(L_, lProf_.pos(segDurationS_), lSentSteps_);
      runAxisTo_(R_, rProf_.pos(segDurationS_), rSentSteps_);
      active_ = false;
    }
  }

  bool isBusy() const { return active_ || (q_.size() > 0); }

private:

  static float wrapDeg_(float a) {
    while (a >= 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
  }


  bool startNext_() {
  MotionCmd cmd;
  if (!q_.pop(cmd)) return false;

  float l_mm = 0.0f, r_mm = 0.0f;

  if (cmd.type == MotionType::Straight) {
    l_mm = cmd.value;
    r_mm = cmd.value;

  } else if (cmd.type == MotionType::Turn) {
    const float angRad = degToRad(cmd.value);
    const float s_mm = angRad * (WHEELBASE_MM * 0.5f); // nominal wheel arc length

    // Nominal steps per wheel (magnitude)
    float s_steps_nom = fabsf(mmToSteps(s_mm));

    // Add calibration: scaled by angle/90
    float scale = fabsf(cmd.value) / 90.0f;
    float bias = (float)TURN90_BIAS_STEPS * scale;

    // Clamp bias
    if (bias > (float)TURN_BIAS_MAX_STEPS) bias = (float)TURN_BIAS_MAX_STEPS;

    float s_steps = s_steps_nom + bias;

    // Convert back to mm so the rest of the pipeline stays unchanged
    float s_mm_cal = s_steps / mmToSteps(1.0f); // mm per step inversion

    // Apply sign (left negative, right positive for +angle)
    const float sign = (cmd.value >= 0.0f) ? 1.0f : -1.0f;
    l_mm = -sign * s_mm_cal;
    r_mm = +sign * s_mm_cal;

  } else if (cmd.type == MotionType::Arc) {
    if (!arcToWheelDists_(cmd.dx_mm, cmd.dy_mm, l_mm, r_mm)) {
      // fallback: if arc solve fails, treat as straight dx
      l_mm = cmd.dx_mm;
      r_mm = cmd.dx_mm;
    }
  }

  const float l_steps = mmToSteps(l_mm);
  const float r_steps = mmToSteps(r_mm);

  lProf_.plan(l_steps, cmd.time_s, maxVelSteps_, maxAccelSteps_);
  rProf_.plan(r_steps, cmd.time_s, maxVelSteps_, maxAccelSteps_);

  segDurationS_ = fmaxf(lProf_.duration(), rProf_.duration());
  segStartUs_   = micros();

  lSentSteps_ = 0;
  rSentSteps_ = 0;

  active_ = true;
  return true;
}
static bool arcToWheelDists_(float dx, float dy, float& l_mm, float& r_mm) {
  const float W = WHEELBASE_MM;

  if (fabsf(dx) < 1e-6f && fabsf(dy) < 1e-6f) {
    l_mm = 0.0f; r_mm = 0.0f;
    return false;
  }
  if (fabsf(dy) < 1e-6f) {
    l_mm = dx; r_mm = dx;
    return true;
  }

  const float R = (dx*dx + dy*dy) / (2.0f * dy);
  const float theta = atan2f(dx, (R - dy));

  l_mm = (R - W*0.5f) * theta;
  r_mm = (R + W*0.5f) * theta;
  return true;
}

  static void runAxisTo_(StepperDriver& drv, float desiredPosSteps, int32_t& sentSteps) {
    // desiredPosSteps is float; sentSteps is the integer number of steps already emitted.
    // If desired is ahead by N steps, emit N steps now (bounded by CPU time).
    int32_t target = (int32_t)floorf(desiredPosSteps + (desiredPosSteps >= 0 ? 1e-6f : -1e-6f));

    while (sentSteps < target) {
      drv.step(+1);
      sentSteps++;
    }
    while (sentSteps > target) {
      drv.step(-1);
      sentSteps--;
    }
  }

  StepperDriver& L_;
  StepperDriver& R_;
  MotionQueue& q_;

  TrapezoidProfile lProf_, rProf_;
  bool active_ = false;
  uint32_t segStartUs_ = 0;
  float segDurationS_ = 0.0f;

  int32_t lSentSteps_ = 0;
  int32_t rSentSteps_ = 0;

  float maxVelSteps_ = 0.0f;
  float maxAccelSteps_ = 0.0f;

  GyroBNO055* gyro_ = nullptr;
  bool segIsTurn_ = false;
  float turnTargetHeadingDeg_ = 0.0f;

};
