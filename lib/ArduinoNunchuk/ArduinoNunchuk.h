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

#ifndef ArduinoNunchuk_H
#define ArduinoNunchuk_H
#include <Arduino.h>

/************************* Nunchuk *******************************/
#define NUNCHUCK_VCCPIN    18      // Pin used to supply Power
#define NUNCHUCK_GNDPIN    19      // Pin used as GND

#define NUNCHUCK_JOYSTICK_THRESHOLD_LOW   77 // calibration values above this are considered invalid
#define NUNCHUCK_JOYSTICK_THRESHOLD_HIGH 177 // calibration values below this are considered invalid
#define NUNCHUCK_JOYSTICK_STEER_MULT     0.5 // 0.8 too much
#define NUNCHUCK_JOYSTICK_SPEED_MULT     0.7 // 0.5 way too slow
#define NUNCHUCK_ACCEL_SPEED_ANGLE        60 // Pitch angle needed to reach full speed (60° with factor 0.5 was too slow)
#define NUNCHUCK_ACCEL_STEER_ANGLE       100 // Pitch angle needed to reach full speed (90° with factor 0.8 was ok,little slow)

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
    int yaw_zero=0, pitch_zero=0, roll_zero=0;

    void init();
    int  update();
    void debug(Stream &port);
    int  update(double &speed, double &steer);
    bool checkID(); 
    bool reInit();

    double rollangle()  { return (atan2(accelX, accelZ) * 180.0 / M_PI); }
    double pitchangle() { return (atan2(accelY, accelZ) * 180.0 / M_PI); }
    double yawangle()   { return (atan2(accelZ, accelX) * 180.0 / M_PI); } // TODO: Check if it is working..

  protected:
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

#endif
