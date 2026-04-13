#include <Arduino.h>
#include <Wire.h>
#include "Config.h"
#include "MotionQueue.h"
#include "StepperDriver.h"
#include "MotionController.h"
#include "Button.h"
#include "GyroMPU6050.h"


// ================= SYSTEM OBJECTS =================
MotionQueue mq;

StepperDriver stepL(PIN_L_STEP, PIN_L_DIR, PIN_ENABLE);
StepperDriver stepR(PIN_R_STEP, PIN_R_DIR, PIN_ENABLE);

TwoWire WireGyro = TwoWire(0);  // order is (SCL, SDA)
GyroMPU6050 gyro(WireGyro);

MotionController motion(stepL, stepR, mq, &gyro);


Button startButton(PIN_START_BUTTON, START_ACTIVE_LOW);


static inline MotionCmd StraightCmd(float dist_mm, float t) {
  MotionCmd c{};
  c.type = MotionType::Straight;
  c.value = dist_mm;
  c.time_s = t;
  return c;
}

static inline MotionCmd TurnCmd(float angle_deg, float t) {
  MotionCmd c{};
  c.type = MotionType::TurnInPlace;
  c.value = angle_deg;
  c.time_s = t;
  return c;
}

static inline MotionCmd ArcCmd(float radius_mm, float angle_deg, float t) {
  MotionCmd c{};
  c.type = MotionType::ArcRadiusAngle;
  c.radius_mm = radius_mm;
  c.angle_deg = angle_deg;
  c.time_s = t;
  return c;
}
void fwd500(){
  mq.push(StraightCmd(500.0f, 1.1f));
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

 //fwd500();
 //left90();
 //fwd500();
 

  //motion.planStraight(700.0f, 3.0f, 0.0f,   50.0f);  // start from rest, speed up
  //motion.planStraight(100, 2, 50, 50);
  //motion.planStraight(500, 2.5, 50, 0);

  //motion.planStraight(400.0f, 6.5f, 300.0f, 120.0f);  // slow down, but do not stop
  //motion.planStraight(100.0f, 6.0f, 120.0f, 120.0f);  
  //motion.planStraight(400.0f, 6.5f, 120.0f, 300.0f);  // slow down, but do not stop
  //motion.planStraight(500.0f, 6.0f, 300.0f, 0.0f);    // finally stop
 
  motion.planStraight(500.0f, 3.0f, 0.0f, 250.0f);

  // Stitch into a left arc without decel: end v of straight = start v of arc
  motion.planArc(250.0f, 90.0f, 3.0f, 250.0f, 250.0f);

  // Stitch to another straight, maybe decel a bit
  motion.planStraight(500.0f, 3.0f, 250.0f, 0.0f);

  //motion.planArc(250.0f, 250.0f, 3.0f, 250.0f, 250.0f);
  

  // End at rest
  //motion.planStraight(500.0f, 2.0f, 100.0f, 0.0f);

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
  delay(1200);


  stepL.begin(false);
  stepR.begin(false);

  Serial.println("Init MPU6050...");
/*
  if (!gyro.begin(MPU6050_ADDR, GYRO_I2C_HZ)) {
    Serial.println("Failed to find MPU6050 on custom I2C bus.");
    while (1) delay(10);
  }

  delay(1000);
  Serial.println("Calibrating gyro bias - keep robot still...");
  gyro.calibrateBias(GYRO_BIAS_SAMPLES, GYRO_BIAS_DELAY_MS);
  Serial.print("Gyro bias (deg/s): ");
  Serial.println(gyro.gyroBiasDegPerS(), 6);
  Serial.println("Gyro ready.");

*/


  stepL.initMicrosteps();
  stepR.initMicrosteps();

  motion.begin();
  startButton.begin();

  Serial.println("RT Ready. Waiting for button...");
}

void loop() {
  //gyro.update();
  motion.update();

  //Serial.println(gyro.headingDeg());

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
