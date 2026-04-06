#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "Config.h"

class GyroMPU6050 {
public:
  GyroMPU6050(TwoWire& wire) : wire_(wire) {}

  bool begin(uint8_t addr = 0x68, uint32_t i2cHz = 100000) {
    wire_.begin(PIN_GYRO_SDA, PIN_GYRO_SCL);
    wire_.setClock(i2cHz);

    if (!mpu_.begin(addr, &wire_)) {
      return false;
    }

    // Good starting settings for turn control
    mpu_.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu_.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu_.setFilterBandwidth(MPU6050_BAND_21_HZ);

    delay(50);
    resetHeading();
    lastMicros_ = micros();
    initialized_ = true;
    return true;
  }
  
  void zeroHeading() {
    headingDeg_ = 0.0f;
    lastMicros_ = micros();
  }

  void resetHeading() {
    headingDeg_ = 0.0f;
    gyroBiasDegPerS_ = 0.0f;
    lastMicros_ = micros();
  }

  // Keep robot still during this
  void calibrateBias(uint16_t samples, uint16_t delayMs) {
    if (!initialized_) return;

    float sum = 0.0f;
    sensors_event_t a, g, t;

    for (uint16_t i = 0; i < samples; i++) {
      mpu_.getEvent(&a, &g, &t);
      // Adafruit gyro units are rad/s
      float gzDegPerS = g.gyro.x * 180.0f / PI;
      sum += gzDegPerS;
      delay(delayMs);
    }

    gyroBiasDegPerS_ = sum / (float)samples;
    headingDeg_ = 0.0f;
    lastMicros_ = micros();
  }

  // Call every loop
  void update() {
    if (!initialized_) return;

    sensors_event_t a, g, t;
    mpu_.getEvent(&a, &g, &t);

    const uint32_t now = micros();
    float dt = (now - lastMicros_) * 1e-6f;
    lastMicros_ = now;

    if (dt <= 0.0f || dt > 0.05f) {
      return;
    }

    float gzDegPerS = g.gyro.x * 180.0f / PI;
    float corrected = gzDegPerS - gyroBiasDegPerS_;
    headingDeg_ += corrected * dt;
    headingDeg_ = wrapDeg_(headingDeg_);
  }

  float headingDeg() const {
    return headingDeg_;
  }

  float gyroBiasDegPerS() const {
    return gyroBiasDegPerS_;
  }

private:
  static float wrapDeg_(float a) {
    while (a >= 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
  }

  TwoWire& wire_;
  Adafruit_MPU6050 mpu_;
  bool initialized_ = false;

  float gyroBiasDegPerS_ = 0.0f;
  float headingDeg_ = 0.0f;
  uint32_t lastMicros_ = 0;
};