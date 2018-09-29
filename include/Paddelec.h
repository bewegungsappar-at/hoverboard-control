#ifndef Paddelec_H
#define Paddelec_H

#ifdef PADDELEC

#include <Arduino.h>
#include "config.h"
#include "Gametrak.h"


class Paddelec 
{
  public:

    Gametrak gametrak1 = Gametrak(GAMETRAK1_RPIN, GAMETRAK1_PHIPIN, GAMETRAK1_THETAPIN, GAMETRAK1_PHI_REV, GAMETRAK1_THETA_REV);
    Gametrak gametrak2 = Gametrak(GAMETRAK2_RPIN, GAMETRAK2_PHIPIN, GAMETRAK2_THETAPIN, GAMETRAK2_PHI_REV, GAMETRAK2_THETA_REV);

    struct PaddelecConfig {
      double paddleAngleThreshold;
      double pwmMultiplier;
      double crosstalkLR;
      double realign;
      double deltaRtoSpeed;
      double drag;
      uint16_t pwmLimit;
      uint16_t steerLimit;
    };
    PaddelecConfig cfgPaddle;
    Paddelec()
    {
      switchOn();
      cfgPaddle.paddleAngleThreshold = 20.0;     // activation angle threshold of paddle. Below threshold, paddle is not enganged and paddelec is freewheeling.
      cfgPaddle.deltaRtoSpeed        =  2.7;     // conversion factor between Gametrak movement to speed. This defines also the maximum speed.
      cfgPaddle.pwmMultiplier        =  0.035;   // effect of paddle stroke to speed
      cfgPaddle.crosstalkLR          =  0.002;   // multiplier for steering
      cfgPaddle.realign              =  0.0005;  // paddelc tries to go straight forward
      cfgPaddle.drag                 =  0.0004;  // drag/water resistance
    }
    void update(double &pwm, double &steer, double &actualSpeed_kmh, double &actualSteer, uint32_t &deltaMillis);
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
      #ifdef GAMETRAK2_GNDPIN
        pinMode(GAMETRAK2_GNDPIN,OUTPUT);
        digitalWrite(GAMETRAK2_GNDPIN,LOW);
      #endif
      #ifdef GAMETRAK2_GNDPIN
        pinMode(GAMETRAK2_VCCPIN,OUTPUT);
        digitalWrite(GAMETRAK2_VCCPIN,HIGH);
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
      #ifdef GAMETRAK2_GNDPIN
        pinMode(GAMETRAK2_GNDPIN,INPUT); // High Z
      #endif
      #ifdef GAMETRAK2_GNDPIN
        pinMode(GAMETRAK2_VCCPIN,INPUT); // High Z
      #endif
    }
    void RLpwmToSteer(double &steer, double &pwm, double &pwmR, double &pwmL);
    void steerToRL(double  &steer, double  &pwm, double  &pwmR, double  &pwmL);
};

#endif
#endif