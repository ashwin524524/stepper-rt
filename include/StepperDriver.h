#pragma once
#include <Arduino.h>
#include "Config.h"

class StepperDriver {
public:
  StepperDriver(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin = 255)
  : stepPin_(stepPin), dirPin_(dirPin), enPin_(enablePin) {}

  void begin(bool invertDir = false) {
    invertDir_ = invertDir;
    pinMode(stepPin_, OUTPUT);
    pinMode(dirPin_, OUTPUT);
    digitalWrite(stepPin_, LOW);
    digitalWrite(dirPin_, LOW);

    if (enPin_ != 255) {
      pinMode(enPin_, OUTPUT);
      disable();   // start disabled
    }
  }
  

  inline void enable() {
    if (enPin_ != 255) {
      digitalWrite(enPin_, LOW);   // LOW = enabled
      enabled_ = true;
    }
  }

  inline void disable() {
    if (enPin_ != 255) {
      digitalWrite(enPin_, HIGH);  // HIGH = disabled
      enabled_ = false;
    }
  }

  inline bool isEnabled() const {
    return enabled_;
  }

  void initMicrosteps(){
    pinMode(MICROSTEP1, OUTPUT);
    pinMode(MICROSTEP2, OUTPUT);


    digitalWrite(MICROSTEP1, LOW);
    digitalWrite(MICROSTEP2, HIGH);
  }

  // Emit exactly one step in the given direction (+1 or -1)
  inline void step(int dir) {
    const bool forward = (dir > 0);
    digitalWrite(dirPin_, (forward ^ invertDir_) ? HIGH : LOW);

    // Short pulse; most drivers need >1us high time
    digitalWrite(stepPin_, HIGH);
    delayMicroseconds(4);
    digitalWrite(stepPin_, LOW);
  }

private:
  uint8_t stepPin_, dirPin_, enPin_;
  bool invertDir_ = false;
  bool enabled_ = false;
};