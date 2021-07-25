#pragma once
#include <Arduino.h>
#include "config.h"

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

extern volatile motorControl motor;
extern bool debug;
extern volatile int32_t deltaMillis;

void slowReset(volatile double &variable, double goal, double step, double fact);
void slowReset(int32_t &variable, double goal, int32_t step, double fact);