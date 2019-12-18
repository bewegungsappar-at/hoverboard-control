#pragma once
/*
 * ArduinoNunchuk.h - Improved Wii Nunchuk library for Arduino
 *
 * Copyright 2011-2013 Gabriel Bianconi, http://www.gabrielbianconi.com/
 *
 * Project URL: http://www.gabrielbianconi.com/projects/arduinonunchuk/
 *
 * Based on the following resources:
 *   http://www.windmeadow.com/node/42
 *   http://todbot.com/blog/2008/02/18/wiichuck-wii-nunchuck-adapter-available/
 *   http://wiibrew.org/wiki/Wiimote/Extension_Controllers
 *
 */

#include <Arduino.h>

#define NUNCHUK_ERR_NOERR  0
#define NUNCHUK_ERR_COUNT  1 // Wrong amount of data read
#define NUNCHUK_ERR_NOINIT 2 // Answer only 0xFF, needs to be initialized
#define NUNCHUK_ERR_SEND   3 // Error during sending
#define NUNCHUK_ERR_ZERO   4 // Answer only 0x00, invalid (all acceleration values 0 is very unlikely)
#define NUNCHUK_ERR_DEV1 101 // 1 value deviates from historic values
#define NUNCHUK_ERR_DEV2 102 // 2 values deviate from historic values
#define NUNCHUK_ERR_DEV3 103 // 3 values deviate from historic values
#define NUNCHUK_ERR_DEV4 104 // 4 values deviate from historic values
#define NUNCHUK_ERR_DEV5 105 // 5 values deviate from historic values
#define NUNCHUK_ERR_DEV6 106 // 6 values deviate from historic values
#define NUNCHUK_ERR_DEV7 107 // 7 values deviate from historic values



/************************* Nunchuk *******************************/
//#define NUNCHUK_VCCPIN    18      // Pin used to supply Power
//#define NUNCHUK_GNDPIN    19      // Pin used as GND

#define NUNCHUK_JOYSTICK_THRESHOLD_LOW   77 // calibration values above this are considered invalid
#define NUNCHUK_JOYSTICK_THRESHOLD_HIGH 177 // calibration values below this are considered invalid
#define NUNCHUK_JOYSTICK_STEER_MULT     0.5 // 0.8 too much
#define NUNCHUK_JOYSTICK_SPEED_MULT     1.4 // 0.5 way too slow
#define NUNCHUK_ACCEL_SPEED_ANGLE        60 // Pitch angle needed to reach full speed (60° with factor 0.5 was too slow)
#define NUNCHUK_ACCEL_STEER_ANGLE       100 // Pitch angle needed to reach full speed (90° with factor 0.8 was ok,little slow)
#define NUNCHUK_HISTORY                   8 // Number of array size for history. Use multiples of 2 to simplify division
#define NUNCHUK_HIST_ANALOGTHRESH        60 // If value deviates by this number, consider invalid
#define NUNCHUK_HIST_ACCELTHRESH         70 // If value deviates by this number, consider invalid
#define NUNCHUK_HIST_BUTTONTHRESH         0 // This many values out of history can deviate to still consider the button press valid

class ArduinoNunchuk
{
  public:
    int analogX, analogX_min=127, analogX_zero=0, analogX_max=127;
    int analogY, analogY_min=127, analogY_zero=0, analogY_max=127;
    int accelX, accelX_start;
    int accelY, accelY_start;
    int accelZ, accelZ_start;
    int zButton, zButton_last;
    int cButton, cButton_last;
    double yaw_zero=0, pitch_zero=0, roll_zero=0;

    void init();
    int  update();
    void debug(Stream &port);
    int  update(volatile double &speed, volatile double &steer);
    bool checkID();
    bool reInit();

    double rollangle()  { return (atan2(accelX, accelZ) * 180.0 / M_PI); }
    double pitchangle() { return (atan2(accelY, accelZ) * 180.0 / M_PI); }
    double yawangle()   { return (atan2(accelZ, accelX) * 180.0 / M_PI); } // TODO: Check if it is working..

  protected:
    int16_t avg_history[7][NUNCHUK_HISTORY];
    long avg_sum[7];
    int avg_ptr;


    void slowReset(int &variable, int goal, int step);

    double scaleAngle(double angle, double factor)    {
      if(angle<-180.0) angle+=360.0;
      else if (angle>180.0) angle-=360.0;
      return angle*factor;
    }
    double limit(double min, double value, double max) {
      if(value<min) value = min;
      if(value>max) value = max;
      return value;
    }

  private:
    uint8_t _sendByte(byte data, byte location);
};