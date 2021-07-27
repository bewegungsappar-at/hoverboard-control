#pragma once

#include <Arduino.h>
#include "config.h"

#ifdef INPUT_IMU_BNO0805
  #include <SparkFun_BNO080_Arduino_Library.h>

  #include <Wire.h>

  #define INT_PIN 32

  // Joystik
  #define JOYPIN_Z  35
  #define JOYPIN_X  34          
  #define JOYPIN_Y  33 


#else
  #include <IMU.h>
#endif

class Paddelec
{
  public:

#ifdef INPUT_IMU_BNO0805
  BNO080 imu;
  
  uint16_t enableActivities  = 0x1F;
  byte activityConfidences[9];
  double bno0805_yaw=0;
  double bno0805_last_yaw=0;
  double bno0805_roll=0;
  double pitchangle_zero =0;
  bool joystik_mode = false;
  bool joystik_mode_change_request = false;
  
#else
  Imu imu=Imu();
#endif    

    struct PaddelecConfig
    {
      double paddleAngleThreshold;
      double pwmMultiplier;
      double crosstalkLR;
      double realign;
      double deltaRtoSpeed;
      double drag;
      int flipControl;
      double maxValidSpeed;
      double maxValidSteer;
      int16_t maxValidGyro;
      double inertiaThreshold;
      double inertiaOffset;
      bool debugMode;
      double pwmLimit;
    };
    PaddelecConfig cfgPaddle;



    void update(volatile double &pwm, volatile double &steer, volatile double &actualSpeed_kmh, volatile double &actualSteer, volatile uint32_t deltaMillis);
    void debug(Stream &port);
    void RLpwmToSteer(volatile double &steer, volatile double &pwm, double &pwmR, double &pwmL);
    void steerToRL(volatile double  &steer, volatile double  &pwm, double  &pwmR, double  &pwmL);
    bool init();

  private:
    void slowReset(volatile double &variable, double goal, double step, double fact);
};