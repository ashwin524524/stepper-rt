#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class GyroBNO055 {
public:
  bool begin(uint8_t addr, uint32_t i2cHz) {
    Wire.begin();
    Wire.setClock(i2cHz);

    if (!bno_.begin(OPERATION_MODE_NDOF)) {
      return false;
    }

    delay(50);
    bno_.setExtCrystalUse(true); // recommended if your breakout has a crystal
    delay(20);

    return true;
  }

  // "Bias" here = heading zero offset (average heading while robot is still)
  void calibrateHeadingZero(uint16_t samples, uint16_t delayMs) {
    float sum = 0.0f;
    float last = readHeadingDegRaw_();
    sum += last;

    for (uint16_t i = 1; i < samples; i++) {
      delay(delayMs);
      float h = readHeadingDegRaw_();
      // unwrap to keep average meaningful across 0/360 boundary
      float dh = wrapDeg_(h - last);
      last = last + dh;
      sum += last;
    }

    float avg = sum / (float)samples;
    headingZeroDeg_ = wrap360_(avg);
  }

  // Heading in degrees, wrapped to [-180, +180) relative to zero
  float headingDeg() {
    float h = readHeadingDegRaw_();
    float rel = wrapDeg_(h - headingZeroDeg_);
    return rel;
  }

  // Useful if you want an absolute-ish heading with wrap [0,360)
  float headingDeg360() {
    float h = readHeadingDegRaw_();
    return wrap360_(h - headingZeroDeg_);
  }

private:
  // BNO055 Euler heading is x() in Adafruit lib (heading, roll, pitch)
  float readHeadingDegRaw_() {
    imu::Vector<3> e = bno_.getVector(Adafruit_BNO055::VECTOR_EULER);
    float heading = e.x(); // degrees, typically [0,360)
    return wrap360_(heading);
  }

  static float wrap360_(float a) {
    while (a >= 360.0f) a -= 360.0f;
    while (a < 0.0f) a += 360.0f;
    return a;
  }

  // wrap to [-180,180)
  static float wrapDeg_(float a) {
    while (a >= 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
  }

  Adafruit_BNO055 bno_{55, 0x28};
  float headingZeroDeg_ = 0.0f;
};
