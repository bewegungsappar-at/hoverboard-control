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


  private:
    uint8_t _sendByte(byte data, byte location);
    void slowReset(int &variable, int goal, int step);
};

#endif
