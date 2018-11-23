#pragma once
#include <Arduino.h>

extern int32_t deltaMillis;

struct motorControl {
  double steer;
  double pwm;     // called "speed" in hoverboard firmware, but is only pwm duty cycle in promille
                  // Values from -1000 to 1000. Negative values represent driving backwards.
  double actualSpeed_kmh;  // motor speed in m/h
  double actualSteer_kmh;  // motor steer
};

extern motorControl motor;
extern bool debug;

#ifdef DEBUG_PLOTTER
  #include "Plotter.h"
  extern Plotter plot;
#endif