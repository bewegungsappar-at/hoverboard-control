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
    int update();
    void debug(Stream &port);
    ArduinoNunchuk ()
    {
     // init(); // TODO: Check re-init. doing two inits seem to bring shitty behaviour 
    }
    int update(int16_t &speed, int16_t &steer);
    bool checkID(); 
    bool reInit();

    int rollangle()
      {
      return (atan2(accelX, accelZ) * 180 / M_PI);
      }
    int pitchangle()
      {
      return (atan2(accelY, accelZ) * 180 / M_PI);
      }
    int yawangle()
      {
      return (atan2(accelZ, accelX) * 180 / M_PI); // TODO: Check if it is working..
      }

  private:
    uint8_t _sendByte(byte data, byte location);
    void slowReset(int &variable, int goal, int step);

    int scaleAngle(int angle, double factor)
    {
      if(angle<-180) angle+=360;
      else if (angle>180) angle-=360;
      return angle*factor;
    }
};

#endif
