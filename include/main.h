#pragma once
#include <Arduino.h>

extern int32_t deltaMillis;

struct motorSetpoint {
  double steer;
  double pwm;     // called "speed" in hoverboard firmware, but is only pwm duty cycle in promille
                  // Values from -1000 to 1000. Negative values represent driving backwards.
};

struct motorMeasured {
  double actualSpeed_kmh;  // motor speed in m/h
  double actualSteer_kmh;  // motor steer
};

struct motorControl {
  motorSetpoint  setpoint;
  motorMeasured measured;
};

extern motorControl motor;
extern bool debug;
extern double plotterTempDouble[4];