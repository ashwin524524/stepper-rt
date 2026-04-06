#pragma once
#include <Arduino.h>

// =================== PINS ===================
static constexpr uint8_t PIN_L_STEP = 8;
static constexpr uint8_t PIN_L_DIR  = 7;
static constexpr uint8_t PIN_R_STEP = 20;
static constexpr uint8_t PIN_R_DIR  = 21;
static constexpr uint8_t PIN_ENABLE = 6;   // optional; set to 255 if unused


static constexpr uint8_t MICROSTEP1 = 9;
static constexpr uint8_t MICROSTEP2 = 10;

// =================== ROBOT GEOMETRY ===================
// Edit these to match your RT
static constexpr float WHEEL_DIAMETER_MM = 127.0f;
static constexpr float WHEELBASE_MM      = 218.0f;  // distance between wheel contact patches

// =================== STEPPER / DRIVE ===================
// motor steps per rev * microsteps (e.g. 200 * 16 = 3200)
static constexpr int32_t STEPS_PER_REV   = 200*4;

// =================== LIMITS (YOU WILL PROVIDE) ===================
// Units are in "steps" domain internally, so we convert mm->steps.
static constexpr float MAX_VEL_MM_S      = 600.0f;   // <-- set this
static constexpr float MAX_ACCEL_MM_S2   = 300.0f;  // <-- set this

// =================== QUEUE ===================
static constexpr size_t MOTION_QUEUE_CAP = 64;

// =================== START BUTTON ===================
static constexpr uint8_t PIN_START_BUTTON = 0;   // <-- you choose
static constexpr bool START_ACTIVE_LOW = false;   // true if using INPUT_PULLUP

// =================== MPU6050 ===================
static constexpr uint8_t MPU6050_ADDR = 0x68;   // usually 0x68, sometimes 0x69
static constexpr uint8_t PIN_GYRO_SDA = 3;
static constexpr uint8_t PIN_GYRO_SCL = 4;

// 100k is a safe starting point
static constexpr uint32_t GYRO_I2C_HZ = 400000;

// Bias calibration
static constexpr uint16_t GYRO_BIAS_SAMPLES = 2400;
static constexpr uint16_t GYRO_BIAS_DELAY_MS = 2;

// Turn correction
static constexpr float TURN_KP_STEPS_PER_DEG = 0.8f;
static constexpr float TURN_CORR_MAX_STEPS = 50.0f;


// =================== TURN CALIBRATION ===================
// Adds (angle_deg/90) * TURN90_BIAS_STEPS to EACH wheel's magnitude during TurnInPlace.
// Positive -> turns more; Negative -> turns less.
static constexpr int32_t TURN90_BIAS_STEPS = 0;   // <-- tune this (+/-)
static constexpr int32_t TURN_BIAS_MAX_STEPS = 800; // safety clamp


// =================== UTILS ===================
inline float mmToSteps(float mm) {
  const float circ = (float)M_PI * WHEEL_DIAMETER_MM;
  return mm * (float)STEPS_PER_REV / circ;
}

inline float degToRad(float deg) {
  return deg * (float)M_PI / 180.0f;
}
