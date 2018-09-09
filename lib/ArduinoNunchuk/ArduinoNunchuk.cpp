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
  /* Power Cycle Nunchuck */
  #ifdef NUNCHUCK_GNDPIN
    pinMode(NUNCHUCK_GNDPIN,INPUT);
  #endif
  #ifdef NUNCHUCK_VCCPIN
    pinMode(NUNCHUCK_VCCPIN,INPUT);
  #endif

  delay(100);
  
  #ifdef NUNCHUCK_GNDPIN
    pinMode(NUNCHUCK_GNDPIN,OUTPUT);
    digitalWrite(NUNCHUCK_GNDPIN,LOW);
  #endif
  #ifdef NUNCHUCK_VCCPIN
    pinMode(NUNCHUCK_VCCPIN,OUTPUT);
    digitalWrite(NUNCHUCK_VCCPIN,HIGH);
  #endif

  delay(100);


  Wire.begin();



  ArduinoNunchuk::_sendByte(0x55, 0xF0);
  ArduinoNunchuk::_sendByte(0x00, 0xFB);

  ArduinoNunchuk::update();
}


bool ArduinoNunchuk::reInit()
{

    /* Power Cycle Nunchuck */
  #ifdef NUNCHUCK_GNDPIN
    pinMode(NUNCHUCK_GNDPIN,INPUT);
  #endif
  #ifdef NUNCHUCK_VCCPIN
    pinMode(NUNCHUCK_VCCPIN,INPUT);
  #endif

  delay(100);
  
  #ifdef NUNCHUCK_GNDPIN
    pinMode(NUNCHUCK_GNDPIN,OUTPUT);
    digitalWrite(NUNCHUCK_GNDPIN,LOW);
  #endif
  #ifdef NUNCHUCK_VCCPIN
    pinMode(NUNCHUCK_VCCPIN,OUTPUT);
    digitalWrite(NUNCHUCK_VCCPIN,HIGH);
  #endif

  delay(100);

  if(!ArduinoNunchuk::_sendByte(0x55, 0xF0)) return false;
  return ArduinoNunchuk::_sendByte(0x00, 0xFB);
}

int ArduinoNunchuk::update()
{
  int error = 0;

  error = ArduinoNunchuk::_sendByte(0x00, 0x00)*100;
  delay(1);

  int count = 0;
  unsigned char values[6];
  int countFF = 0;
  if(Wire.requestFrom(ADDRESS, 6) != 6) error++;

  while(Wire.available())
  {
    values[count] = Wire.read();
    if(values[count] == 0xFF) countFF++;
    count++;
  }
  if(count!=6) error++;

  if(countFF<6 && error == 0)  // if all Bytes are FF, the Nunchuck needs to be initialised probably. Errors indicate communication problems.
  {
    /* valid set received */
    cButton_last = cButton;
    zButton_last = zButton;

    ArduinoNunchuk::analogX = values[0];
    ArduinoNunchuk::analogY = values[1];
    ArduinoNunchuk::accelX = ((values[2] << 2) | ((values[5] >> 2) & 3))-511;
    ArduinoNunchuk::accelY = ((values[3] << 2) | ((values[5] >> 4) & 3))-511;
    ArduinoNunchuk::accelZ = ((values[4] << 2) | ((values[5] >> 6) & 3))-511;
    ArduinoNunchuk::zButton = !((values[5] >> 0) & 1);
    ArduinoNunchuk::cButton = !((values[5] >> 1) & 1);
  } else 
  {
    /* something went wrong - slowly reset everything to safe values */
    slowReset(analogX, analogX_zero, 1);
    slowReset(analogY, analogY_zero, 1);
    slowReset(accelX, accelX_start, 5);
    slowReset(accelY, accelY_start, 5);
    slowReset(accelZ, accelZ_start, 5);
    zButton = 0;
//    cButton = 0; // cButton is used to switch between acceleration mode and joystick mode
  }
  if(countFF == 6) return error + 1000;
  else return error;
}

void ArduinoNunchuk::slowReset(int &variable, int goal, int step) {
  if      ((variable - goal) > step) variable -= step;
  else if ((goal - variable) > step) variable += step;
  else                               variable  = goal;
}

int ArduinoNunchuk::update(int16_t &speed, int16_t &steer)
{
  int error = 0;
  error = update();

  if(cButton && !zButton) 
  /* acceleration control mode when cButton is pressed */
  {
    if(cButton_last != cButton)
    {
      /* use current position as zero */
      accelX_start = accelX;
      accelY_start = accelY;
      accelZ_start = accelZ;
      pitch_zero   = pitchangle();
      yaw_zero     = yawangle();
      roll_zero    = rollangle();
    }
    steer = scaleAngle(rollangle()  - roll_zero , 1000.0 / NUNCHUCK_ACCEL_STEER_ANGLE);
    speed = scaleAngle(pitchangle() - pitch_zero, 1000.0 / NUNCHUCK_ACCEL_SPEED_ANGLE);
  } else if (cButton && zButton)
  /* Joystick calibration mode when both buttons are pressed */
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
  /* use Joystick as Input */
  {
    // check if calib is plausible 
    if(analogX_min  < NUNCHUCK_JOYSTICK_THRESHOLD_LOW  && analogX_max  > NUNCHUCK_JOYSTICK_THRESHOLD_HIGH 
    && analogY_min  < NUNCHUCK_JOYSTICK_THRESHOLD_LOW  && analogY_max  > NUNCHUCK_JOYSTICK_THRESHOLD_HIGH 
    && analogX_zero > NUNCHUCK_JOYSTICK_THRESHOLD_LOW  && analogX_zero < NUNCHUCK_JOYSTICK_THRESHOLD_HIGH 
    && analogY_zero > NUNCHUCK_JOYSTICK_THRESHOLD_LOW  && analogY_zero < NUNCHUCK_JOYSTICK_THRESHOLD_HIGH) 
    {
      if((analogX - analogX_zero)>0) steer = (analogX - analogX_zero) * 1000.0/(analogX_max - analogX_zero) * NUNCHUCK_JOYSTICK_STEER_MULT;
      else                           steer = (analogX - analogX_zero) * 1000.0/(analogX_zero - analogX_min) * NUNCHUCK_JOYSTICK_STEER_MULT;

      if((analogY - analogY_zero)>0) speed = (analogY - analogY_zero) * 1000.0/(analogY_max - analogY_zero) * NUNCHUCK_JOYSTICK_SPEED_MULT;
      else                           speed = (analogY - analogY_zero) * 1000.0/(analogY_zero - analogY_min) * NUNCHUCK_JOYSTICK_SPEED_MULT;

    } else {
      steer = 0;
      speed = 0;
    }
  }
  return error;
}

uint8_t ArduinoNunchuk::_sendByte(byte data, byte location)
{
  Wire.beginTransmission(ADDRESS);

  if(Wire.write(location) != 1) return 9;
  if(Wire.write(data) != 1) return 9;

  return Wire.endTransmission();
//  delay(10); //TODO was: 10ms, is it necessary?
}

void ArduinoNunchuk::debug(Stream &port)
{
  port.print("N ");
  port.printf("%4i %4i | ", analogX, analogY);
  port.printf("%4i %4i %4i | ", accelX, accelY, accelZ);
  port.printf("%2i %2i  ", zButton, cButton);
}

bool ArduinoNunchuk::checkID() 
{
  int count = 0;
  unsigned char values[4];
  unsigned char id[4] = {255,0,164,32};
  bool compare = true;
  Wire.beginTransmission(ADDRESS); // device address
  Wire.write((uint8_t)0xFA);
  Wire.endTransmission();
  delay(1);
  Wire.requestFrom(ADDRESS, 4);
  while(Wire.available())
  {
    values[count] = Wire.read();
  //  port.printf("%3i ", values[count]);
  //  port.printf("%3i ", id[count]);
    if(values[count]!=id[count]) compare = false;
    count++;
  }
  if(compare) return true;
  else return false;
}