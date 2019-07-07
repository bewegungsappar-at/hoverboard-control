#pragma once

#include <Arduino.h>
#include "config.h"
#include <IMU.h>

#if defined(INPUT_PADDELECIMU)

class Paddelec
{
  public:

    Imu imu=Imu();

    struct PaddelecConfig {
      double paddleAngleThreshold;
      double pwmMultiplier;
      double crosstalkLR;
      double realign;
      double deltaRtoSpeed;
      double drag;
      uint16_t pwmLimit;
      uint16_t steerLimit;
      int flipControl;
    };
    PaddelecConfig cfgPaddle;
    bool init() {
      imu.init();
      cfgPaddle.paddleAngleThreshold = 22.0;     // activation angle threshold of paddle. Below threshold, paddle is not enganged and paddelec is freewheeling.
      cfgPaddle.deltaRtoSpeed        =  0.0023;     // conversion factor between Gametrak movement to speed. This defines also the maximum speed.
      cfgPaddle.pwmMultiplier        =  0.023;   // effect of paddle stroke to speed
      cfgPaddle.crosstalkLR          =  0.0;   // multiplier for steering
      cfgPaddle.realign              =  0.0;  // paddelc tries to go straight forward
      cfgPaddle.drag                 =  0.00013;  // drag/water resistance
      cfgPaddle.flipControl          = 1;       // 1: Normal. -1 Flipped

      delay(1000);                  // Wait till shaking from switching on is gone
      imu.pitch_zero = imu.pitchangle();
      return(true);
    }
    void update(volatile double &pwm, volatile double &steer, volatile double &actualSpeed_kmh, volatile double &actualSteer, volatile uint32_t deltaMillis);
    void debug(Stream &port);
    void RLpwmToSteer(volatile double &steer, volatile double &pwm, double &pwmR, double &pwmL);
    void steerToRL(volatile double  &steer, volatile double  &pwm, double  &pwmR, double  &pwmL);

  private:
    void slowReset(volatile double &variable, double goal, double step);
};

#endif