#pragma once
#include <Arduino.h>

// =================== PINS ===================
static constexpr uint8_t PIN_L_STEP = 3;
static constexpr uint8_t PIN_L_DIR  = 2;
static constexpr uint8_t PIN_R_STEP = 7;
static constexpr uint8_t PIN_R_DIR  = 6;
static constexpr uint8_t PIN_ENABLE = 255;   // optional; set to 255 if unused

static constexpr uint8_t MICROSTEP0L = 5;
static constexpr uint8_t MICROSTEP1L = 4;

static constexpr uint8_t MICROSTEP0R = 9;
static constexpr uint8_t MICROSTEP1R = 8;

// =================== ROBOT GEOMETRY ===================
// Edit these to match your RT
static constexpr float WHEEL_DIAMETER_MM = 96.0f;
static constexpr float WHEELBASE_MM      = 195.0f;  // distance between wheel contact patches

// =================== STEPPER / DRIVE ===================
// motor steps per rev * microsteps (e.g. 200 * 16 = 3200)
static constexpr int32_t STEPS_PER_REV   = 200*2;

// =================== LIMITS (YOU WILL PROVIDE) ===================
// Units are in "steps" domain internally, so we convert mm->steps.
static constexpr float MAX_VEL_MM_S      = 600.0f;   // <-- set this
static constexpr float MAX_ACCEL_MM_S2   = 600.0f;  // <-- set this

// =================== QUEUE ===================
static constexpr size_t MOTION_QUEUE_CAP = 64;

// =================== START BUTTON ===================
static constexpr uint8_t PIN_START_BUTTON = A1;   // <-- you choose
static constexpr bool START_ACTIVE_LOW = false;   // true if using INPUT_PULLUP

// =================== BNO055 ===================
static constexpr uint8_t BNO055_ADDR = 0x28;   // 0x28 or 0x29 depending on ADR pin
static constexpr uint32_t I2C_HZ = 400000;

// =================== TURN P CONTROL ===================
// "How hard" to correct heading during turns.
// Start small (e.g. 0.2 to 1.0) and increase until turns tighten up.
static constexpr float TURN_KP_STEPS_PER_DEG = 0.5f;

// Clamp correction so it doesn't go crazy (in steps of extra differential offset)
static constexpr float TURN_CORR_MAX_STEPS = 200.0f;

// =================== STARTUP GYRO ZEROING ===================
static constexpr uint16_t GYRO_BIAS_SAMPLES = 200;   // ~1-2 seconds worth
static constexpr uint16_t GYRO_BIAS_DELAY_MS = 5;    // sample spacing

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
