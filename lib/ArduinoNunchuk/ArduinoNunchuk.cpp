/*
 * ArduinoNunchuk.cpp - Improved Wii Nunchuk library for Arduino
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
#include <Wire.h>
#include "ArduinoNunchuk.h"

#define ADDRESS 0x52

void ArduinoNunchuk::init()
{
  Wire.begin();

  ArduinoNunchuk::_sendByte(0x55, 0xF0);
  ArduinoNunchuk::_sendByte(0x00, 0xFB);

  ArduinoNunchuk::update();
}

void ArduinoNunchuk::update()
{
  int count = 0;
  unsigned char values[6];

  Wire.requestFrom(ADDRESS, 6);

  while(Wire.available())
  {
    values[count] = Wire.read();
    count++;
  }

  cButton_last = cButton;
  zButton_last = zButton;

  ArduinoNunchuk::analogX = values[0];
  ArduinoNunchuk::analogY = values[1];
  ArduinoNunchuk::accelX = (values[2] << 2) | ((values[5] >> 2) & 3);
  ArduinoNunchuk::accelY = (values[3] << 2) | ((values[5] >> 4) & 3);
  ArduinoNunchuk::accelZ = (values[4] << 2) | ((values[5] >> 6) & 3);
  ArduinoNunchuk::zButton = !((values[5] >> 0) & 1);
  ArduinoNunchuk::cButton = !((values[5] >> 1) & 1);

  ArduinoNunchuk::_sendByte(0x00, 0x00);
}

void ArduinoNunchuk::update(int16_t &speed, int16_t &steer)
{
  update();

  // TODO convert to degrees for nunchuck, this is just a hack
  if(cButton && !zButton) 
  {
    if(cButton_last != cButton)
    {
      /* use current position as zero */
      accelX_start = accelX;
      accelY_start = accelY;
      accelZ_start = accelZ;
    }
    steer = (accelX - accelX_start) * 4;
    speed = (accelY - accelY_start) * 4;
  } else if (cButton && zButton)
  {
    if((zButton_last != zButton) || (cButton_last != cButton))  // do calibration
    {
      /* revoke old calibration, set zero */
      analogX_zero = analogX;
      analogY_zero = analogY;
      analogX_min = 127;
      analogX_max = 127;
      analogY_min = 127;
      analogY_max = 127;
    } else {
      /* find extremes */
      if(analogX < analogX_min) analogX_min = analogX;
      if(analogY < analogY_min) analogY_min = analogY;
      if(analogX > analogX_max) analogX_max = analogX;
      if(analogY > analogY_max) analogY_max = analogY;
    }
    steer = 0;
    speed = 0;
  } else
  {
    if(analogX_min<77 && analogY_min<77 && analogX_max>177 && analogY_max>177 && analogX_zero>77 && analogX_zero<177 && analogY_zero>77 && analogY_zero<177) // check if calib is plausible
    {
      if((analogX - analogX_zero)>0) steer = (analogX - analogX_zero) * 1000/(analogX_max - analogX_zero);
      else                           steer = (analogX - analogX_zero) * 1000/(analogX_zero - analogX_min);

      if((analogY - analogY_zero)>0) speed = (analogY - analogY_zero) * 1000/(analogY_max - analogY_zero);
      else                           speed = (analogY - analogY_zero) * 1000/(analogY_zero - analogY_min);

    } else {
      steer = 0;
      speed = 0;
    }
  }
}

void ArduinoNunchuk::_sendByte(byte data, byte location)
{
  Wire.beginTransmission(ADDRESS);

  Wire.write(location);
  Wire.write(data);

  Wire.endTransmission();

  delay(10);
}

void ArduinoNunchuk::debug(Stream &port)
{
  port.print("N ");
  port.printf("%4i %4i | ", analogX, analogY);
  port.printf("%4i %4i %4i | ", accelX, accelY, accelZ);
  port.printf("%2i %2i  ", zButton, cButton);
}