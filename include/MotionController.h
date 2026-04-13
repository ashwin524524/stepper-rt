#pragma once
#include <Arduino.h>
#include "Config.h"
#include "MotionQueue.h"
#include "StepperDriver.h"
#include "TrapezoidProfile.h"
#include "GyroMPU6050.h"
#include "TurnPlanner.h"


// Runs queued motion commands by planning left/right step profiles.
// Emits steps by sampling profile position and stepping until actual step index matches.
class MotionController {
public:
    MotionController(StepperDriver& left, StepperDriver& right, MotionQueue& q, GyroMPU6050* gyro = nullptr)
        : L_(left), R_(right), q_(q), gyro_(gyro) {}


  void begin() {
    maxVelSteps_  = mmToSteps(MAX_VEL_MM_S);
    maxAccelSteps_= mmToSteps(MAX_ACCEL_MM_S2);
  }

  void update() {
    if (!active_) {
      if (!startNext_()) {
        disableMotors_();
        return;
      }
    }

    const uint32_t nowUs = micros();
    const float t = (nowUs - segStartUs_) * 1e-6f;

    // Desired positions from profile (in steps, can be negative)
    const float lPos = lProf_.pos(t);
    const float rPos = rProf_.pos(t);

    float lCmd = lPos;
    float rCmd = rPos;

    
    if (segIsTurn_ && gyro_) {
      float thetaCmd = turnPlanner_.pos(t);   // commanded relative angle at current time
      float headingCmd = wrapDeg_(turnStartHeadingDeg_ + thetaCmd);

      float headingMeas = gyro_->headingDeg();
      float err = wrapDeg_(headingCmd - headingMeas);

      Serial.print(" cmd: "); Serial.print(headingCmd);
      Serial.print(" pos: "); Serial.print(headingMeas);
      Serial.print(" err: "); Serial.println(err);

      float corr = TURN_KP_STEPS_PER_DEG * err;
      if (corr > TURN_CORR_MAX_STEPS) corr = TURN_CORR_MAX_STEPS;
      if (corr < -TURN_CORR_MAX_STEPS) corr = -TURN_CORR_MAX_STEPS;

      //lCmd -= corr;
      //rCmd += corr;
    }
    
    
    //Serial.println(t);
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

      if (q_.size() == 0) {
        disableMotors_();
      }
    }
  }

  // Enqueue a straight with boundary velocities (mm/s)
  bool planStraight(float dist_mm, float time_s, float v0_mm_s, float v1_mm_s) {

    Serial.println("Started new straight");
    MotionCmd c{};
    c.type = MotionType::Straight;
    c.value = dist_mm;
    c.dx_mm = 0; c.dy_mm = 0;
    c.time_s = time_s;
    c.v0_mm_s = v0_mm_s;
    c.v1_mm_s = v1_mm_s;
    return q_.push(c);
  }

  // Enqueue an arc (dx,dy) with boundary CENTER velocities (mm/s)
  bool planArc(float dx_mm, float dy_mm, float time_s, float v0_center_mm_s, float v1_center_mm_s) {
    MotionCmd c{};
    c.type = MotionType::Arc;
    c.value = 0;
    c.dx_mm = dx_mm;
    c.dy_mm = dy_mm;
    c.time_s = time_s;
    c.v0_mm_s = v0_center_mm_s;
    c.v1_mm_s = v1_center_mm_s;
    return q_.push(c);
  }

  bool isBusy() const { return active_ || (q_.size() > 0); }

private:

  static float wrapDeg_(float a) {
    while (a >= 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
  }

  void enableMotors_() {
  L_.enable();
  R_.enable();
}

void disableMotors_() {
  L_.disable();
  R_.disable();
}


  bool startNext_() {
  MotionCmd cmd;
  if (!q_.pop(cmd)) return false;

    enableMotors_();

  float l_mm = 0.0f, r_mm = 0.0f;
    float l_v0 = 0.0f, r_v0 = 0.0f;
    float l_v1 = 0.0f, r_v1 = 0.0f;

  if (cmd.type == MotionType::Straight) {
    l_mm = cmd.value;
    r_mm = cmd.value;
      l_v0 = cmd.v0_mm_s; r_v0 = cmd.v0_mm_s;
      l_v1 = cmd.v1_mm_s; r_v1 = cmd.v1_mm_s;
      segIsTurn_ = false;

    } else if (cmd.type == MotionType::TurnInPlace) {
    const float angRad = degToRad(cmd.value);
      const float s_mm = angRad * (WHEELBASE_MM * 0.5f);

      l_mm = -s_mm;
      r_mm = +s_mm;

      segIsTurn_ = true;

      if (gyro_) {
        turnStartHeadingDeg_ = gyro_->headingDeg();
      } else {
        turnStartHeadingDeg_ = 0.0f;
      }

      turnPlanner_.plan(cmd.value, cmd.time_s);

    } else { // Arc
      float kL=1.0f, kR=1.0f; // wheel speed scaling vs center speed
      if (!arcToWheelDistsAndScales_(cmd.dx_mm, cmd.dy_mm, l_mm, r_mm, kL, kR)) {
        // fallback: straight dx
        l_mm = cmd.dx_mm; r_mm = cmd.dx_mm;
        kL = kR = 1.0f;
      }
      // Boundary center velocities scaled to wheels
      l_v0 = kL * cmd.v0_mm_s;  r_v0 = kR * cmd.v0_mm_s;
      l_v1 = kL * cmd.v1_mm_s;  r_v1 = kR * cmd.v1_mm_s;
      segIsTurn_ = false;
    }

    // Convert to steps domain
  const float l_steps = mmToSteps(l_mm);
  const float r_steps = mmToSteps(r_mm);

    const float l_v0_steps = mmToSteps(l_v0);
    const float r_v0_steps = mmToSteps(r_v0);
    const float l_v1_steps = mmToSteps(l_v1);
    const float r_v1_steps = mmToSteps(r_v1);

    lProf_.plan(l_steps, cmd.time_s, l_v0_steps, l_v1_steps, maxVelSteps_, maxAccelSteps_);
    rProf_.plan(r_steps, cmd.time_s, r_v0_steps, r_v1_steps, maxVelSteps_, maxAccelSteps_);

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

  // Returns wheel distances (mm) and wheel speed scale factors kL,kR such that:
// vL = kL * vCenter, vR = kR * vCenter
  static bool arcToWheelDistsAndScales_(float dx, float dy,
                                      float& l_mm, float& r_mm,
                                      float& kL, float& kR)
  {
    const float W = WHEELBASE_MM;

    if (fabsf(dx) < 1e-6f && fabsf(dy) < 1e-6f) {
      l_mm = r_mm = 0.0f;
      kL = kR = 1.0f;
      return false;
    }

    if (fabsf(dy) < 1e-6f) {
      // straight
      l_mm = dx; r_mm = dx;
      kL = kR = 1.0f;
      return true;
    }

    // Radius R and sweep theta
    const float R = (dx*dx + dy*dy) / (2.0f * dy);
    const float theta = atan2f(dx, (R - dy)); // radians

    // Center arc length
    const float Lc = R * theta;

    // Wheel lengths
    l_mm = (R - W*0.5f) * theta;
    r_mm = (R + W*0.5f) * theta;

    // Speed scale factors vs center speed (avoid divide by zero if R tiny)
    const float Rabs = (fabsf(R) < 1e-6f) ? (R >= 0 ? 1e-6f : -1e-6f) : R;
    kL = (R - W*0.5f) / Rabs;
    kR = (R + W*0.5f) / Rabs;

    // Keep direction consistent with Lc sign. If center arc length is negative (reverse),
    // kL/kR should not flip sign (velocity sign comes from vCenter).
    (void)Lc;
    return true;
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

  GyroMPU6050* gyro_ = nullptr;
  float turnStartHeadingDeg_ = 0.0f;


  TurnPlanner turnPlanner_;
  bool segIsTurn_ = false;
  float turnTargetHeadingDeg_ = 0.0f;

};
