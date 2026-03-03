#include <Arduino.h>
#include "Config.h"
#include "MotionQueue.h"
#include "StepperDriver.h"
#include "MotionController.h"
#include "Button.h"
#include "GyroBNO055.h"


// ================= SYSTEM OBJECTS =================
MotionQueue mq;

StepperDriver stepL(PIN_L_STEP, PIN_L_DIR, PIN_ENABLE);
StepperDriver stepR(PIN_R_STEP, PIN_R_DIR, PIN_ENABLE);

GyroBNO055 gyro;
MotionController motion(stepL, stepR, mq, &gyro);


Button startButton(PIN_START_BUTTON, START_ACTIVE_LOW);

static inline MotionCmd StraightCmd(float dist_mm, float t) {
  return {MotionType::Straight, dist_mm, 0, 0, t};
}
static inline MotionCmd TurnCmd(float deg, float t) {
  return {MotionType::TurnInPlace, deg, 0, 0, t};
}
static inline MotionCmd ArcCmd(float dx, float dy, float t) {
  return {MotionType::Arc, 0, dx, dy, t};
}

void fwd500(){
  mq.push(StraightCmd(500.0f, 1.8f));
}

void back500() {
  mq.push(StraightCmd(-500.0f, 1.8f));
}

void left90(){
  mq.push(TurnCmd(-90.0f, 1.2f));
}
void right90(){
  mq.push(TurnCmd(90.0f, 1.2f));
}

// ================= ROUTINE =================
static void loadRoutine() {
 // Start from rest -> accelerate while going straight
  motion.planStraight(800.0f, 2.5f, 0.0f, 250.0f);

  // Stitch into a left arc without decel: end v of straight = start v of arc
  motion.planArc(400.0f, 200.0f, 1.6f, 250.0f, 250.0f);

  // Stitch to another straight, maybe decel a bit
  motion.planStraight(600.0f, 2.0f, 250.0f, 100.0f);

  // End at rest
  motion.planStraight(200.0f, 1.0f, 100.0f, 0.0f);

}



// ================= STATE MACHINE =================
enum class RunState {
  WaitingForStart,
  Running,
  Finished
};

RunState state = RunState::WaitingForStart;


void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("Init BNO055...");
  if (!gyro.begin(BNO055_ADDR, I2C_HZ)) {
    Serial.println("BNO055 not detected. Check wiring/address.");
    // You can halt here or continue without gyro:
    // while (1) delay(10);
  } else {
    Serial.println("Calibrating heading zero (keep robot still)...");
    gyro.calibrateHeadingZero(GYRO_BIAS_SAMPLES, GYRO_BIAS_DELAY_MS);
    Serial.println("Gyro zeroed.");
  }

  if (PIN_ENABLE != 255) {
    pinMode(PIN_ENABLE, OUTPUT);
    digitalWrite(PIN_ENABLE, LOW);
  }

  stepL.begin(false);
  stepR.begin(false);

  stepL.initMicrosteps();
  stepR.initMicrosteps();

  motion.begin();
  startButton.begin();

  Serial.println("RT Ready. Waiting for button...");
}

void loop() {
  motion.update();

  Serial.println(gyro.headingDeg());

  switch (state) {

    case RunState::WaitingForStart:
      if (startButton.pressed()) {
        Serial.println("Start pressed!");
        loadRoutine();
        state = RunState::Running;
      }
      break;

    case RunState::Running:
      if (!motion.isBusy()) {
        Serial.println("Run complete.");
        state = RunState::Finished;
      }
      break;

    case RunState::Finished:
      // Wait for release before re-arming
      if (startButton.pressed()) {
        Serial.println("Re-armed.");
        state = RunState::WaitingForStart;
      }
      break;
  }
}
