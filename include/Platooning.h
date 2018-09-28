#ifndef Platooning_H
#define Platooning_H

#include <Arduino.h>
#include "config.h"
#include "Gametrak.h"


class Platooning 
{
  public:

    Gametrak gametrak1 = Gametrak(GAMETRAK1_RPIN, GAMETRAK1_PHIPIN, GAMETRAK1_THETAPIN, GAMETRAK1_PHI_REV, GAMETRAK1_THETA_REV);
   
    struct PlatooningConfig {
      double rActivationThreshold_mm;
      double rForwardThreshold_mm;
      double r_mm_forwardToPWM;
      double r_mm_backwardToPWM;
      double phi_deg_ToSteer;
      double PWMforwardLimit;
      double PWMbackwardLimit;
      double SteerLimit;
    };
    PlatooningConfig cfgPlatooning;
    Platooning()
    {
      switchOn();
      cfgPlatooning.rActivationThreshold_mm = 1000.0;  // activation threshold of Gametrak. Do nothing when distance is shorter
      cfgPlatooning.rForwardThreshold_mm    = 1000.0;  // Forward hreshold of Gametrak. below is backwards, above is forward
      cfgPlatooning.r_mm_forwardToPWM       = 1.0;     // Conversion of distance to PWM
      cfgPlatooning.r_mm_backwardToPWM      = 1.0;     // Conversion of distance to PWM
      cfgPlatooning.phi_deg_ToSteer         = 1.0;     // Conversion of steering angle to PWM
      cfgPlatooning.PWMforwardLimit         = 500.0;   // Limit PWM
      cfgPlatooning.PWMbackwardLimit        = 100.0;  // Limit PWM
      cfgPlatooning.SteerLimit              = 500.0;   // Limit Steering
    }
    void update(double &pwm, double &steer);
    void debug(Stream &port);
    
  private: 
    void switchOn()  // TODO Integrate into Gametrak class
    {
      #ifdef GAMETRAK1_GNDPIN
        pinMode(GAMETRAK1_GNDPIN,OUTPUT);
        digitalWrite(GAMETRAK1_GNDPIN,LOW);
      #endif
      #ifdef GAMETRAK1_VCCPIN
        pinMode(GAMETRAK1_VCCPIN,OUTPUT);
        digitalWrite(GAMETRAK1_VCCPIN,HIGH);
      #endif
    }
    
    void switchOff() // TODO Integrate into Gametrak class
    {
      #ifdef GAMETRAK1_GNDPIN
        pinMode(GAMETRAK1_GNDPIN,INPUT); // High Z
      #endif
      #ifdef GAMETRAK1_VCCPIN
        pinMode(GAMETRAK1_VCCPIN,INPUT); // High Z
      #endif
    }

    double limit(double min, double value, double max) {
      if(value<min) value = min;
      if(value>max) value = max;
      return value;
    }
};

#endif