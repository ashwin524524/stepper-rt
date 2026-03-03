#pragma once
#include <Arduino.h>

class Button {
public:
  Button(uint8_t pin, bool activeLow = true)
  : pin_(pin), activeLow_(activeLow) {}

  void begin() {
    if (activeLow_) {
      pinMode(pin_, INPUT_PULLUP);
    } else {
      pinMode(pin_, INPUT);
    }
    lastStableState_ = readRaw_();
    lastReadState_ = lastStableState_;
    lastDebounceTime_ = millis();
  }

  // Returns true ONCE per press (rising edge of logical pressed)
  bool pressed() {
    bool reading = readRaw_();

    if (reading != lastReadState_) {
      lastDebounceTime_ = millis();
    }

    if ((millis() - lastDebounceTime_) > debounceMs_) {
      if (reading != lastStableState_) {
        lastStableState_ = reading;

        if (lastStableState_ == true) {
          lastReadState_ = reading;
          return true; // edge detected
        }
      }
    }

    lastReadState_ = reading;
    return false;
  }

private:
  bool readRaw_() {
    bool raw = digitalRead(pin_);
    return activeLow_ ? !raw : raw;
  }

  uint8_t pin_;
  bool activeLow_;

  bool lastStableState_;
  bool lastReadState_;
  unsigned long lastDebounceTime_ = 0;
  const unsigned long debounceMs_ = 25;
};
