#pragma once
#include <Arduino.h>

class StepperTimer {
public:
  static constexpr uint32_t ISR_HZ = 20000;   // 20 kHz is a good starting point
  static constexpr float DT = 1.0f / ISR_HZ;

  void begin(uint8_t lStep, uint8_t lDir, uint8_t rStep, uint8_t rDir) {
    lStepPin_ = lStep;
    lDirPin_  = lDir;
    rStepPin_ = rStep;
    rDirPin_  = rDir;

    pinMode(lStepPin_, OUTPUT);
    pinMode(lDirPin_, OUTPUT);
    pinMode(rStepPin_, OUTPUT);
    pinMode(rDirPin_, OUTPUT);

    digitalWrite(lStepPin_, LOW);
    digitalWrite(rStepPin_, LOW);
    digitalWrite(lDirPin_, LOW);
    digitalWrite(rDirPin_, LOW);

    instance_ = this;

    timer_ = timerBegin(ISR_HZ);
    timerAttachInterrupt(timer_, &StepperTimer::onTimerISR);
    timerAlarmEnable(timer_, 1, true, 0);  // fire every timer tick
  }

  void stop() {
    noInterrupts();
    targetLStepsPerS_ = 0.0f;
    targetRStepsPerS_ = 0.0f;
    interrupts();
  }

  void setTargetRates(float lStepsPerS, float rStepsPerS) {
    noInterrupts();
    targetLStepsPerS_ = lStepsPerS;
    targetRStepsPerS_ = rStepsPerS;
    interrupts();
  }

private:
  static void IRAM_ATTR onTimerISR() {
    if (instance_) instance_->handleISR_();
  }

  void IRAM_ATTR handleISR_() {
    float lRate = targetLStepsPerS_;
    float rRate = targetRStepsPerS_;

    // direction pins
    bool lForward = (lRate >= 0.0f);
    bool rForward = (rRate >= 0.0f);

    digitalWrite(lDirPin_, lForward ? HIGH : LOW);
    digitalWrite(rDirPin_, rForward ? HIGH : LOW);

    // accumulate fractional steps
    lAccum_ += fabsf(lRate) * DT;
    rAccum_ += fabsf(rRate) * DT;

    // emit at most one step per axis per ISR tick
    // keeps ISR bounded and deterministic
    if (lAccum_ >= 1.0f) {
      digitalWrite(lStepPin_, HIGH);
      digitalWrite(lStepPin_, LOW);
      lAccum_ -= 1.0f;
    }

    if (rAccum_ >= 1.0f) {
      digitalWrite(rStepPin_, HIGH);
      digitalWrite(rStepPin_, LOW);
      rAccum_ -= 1.0f;
    }
  }

  inline static StepperTimer* instance_ = nullptr;

  hw_timer_t* timer_ = nullptr;

  volatile float targetLStepsPerS_ = 0.0f;
  volatile float targetRStepsPerS_ = 0.0f;

  float lAccum_ = 0.0f;
  float rAccum_ = 0.0f;

  uint8_t lStepPin_ = 255, lDirPin_ = 255;
  uint8_t rStepPin_ = 255, rDirPin_ = 255;
};