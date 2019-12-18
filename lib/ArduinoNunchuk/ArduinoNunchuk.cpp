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
#include "protocol.h"
#include "serialbridge.h"

#define ADDRESS 0x52

#ifdef DEBUG_PLOTTER
  #include <Plotter.h>
  extern Plotter plot;

  int avg_diff[5];
#endif

extern volatile PROTOCOL_BUZZER_DATA sendBuzzer;

void ArduinoNunchuk::init()
{
  /* Power Cycle Nunchuk */
  #ifdef NUNCHUK_GNDPIN
    pinMode(NUNCHUK_GNDPIN,INPUT);
  #endif
  #ifdef NUNCHUK_VCCPIN
    pinMode(NUNCHUK_VCCPIN,INPUT);
  #endif

  delay(100);

  #ifdef NUNCHUK_GNDPIN
    pinMode(NUNCHUK_GNDPIN,OUTPUT);
    digitalWrite(NUNCHUK_GNDPIN,LOW);
  #endif
  #ifdef NUNCHUK_VCCPIN
    pinMode(NUNCHUK_VCCPIN,OUTPUT);
    digitalWrite(NUNCHUK_VCCPIN,HIGH);
  #endif

  // initialize array used for sliding average
  for(int i=0; i<7; i++) {
    for(int j; j<NUNCHUK_HISTORY; j++) {
      avg_history[i][j] = 0;
    }
    avg_sum[i] = 0;
  }
  avg_ptr=0;

  delay(100);

#if defined(NUNCHUK_SDAPIN) || defined(NUNCHUK_SCLPIN)
  Wire.begin(NUNCHUK_SDAPIN, NUNCHUK_SCLPIN);
#else
  Wire.begin();
#endif


  ArduinoNunchuk::_sendByte(0x55, 0xF0);
  ArduinoNunchuk::_sendByte(0x00, 0xFB);

#ifdef DEBUG_PLOTTER
//  plot.AddTimeGraph( "Nunchuk Acceleration + diff", 500, "X filtered", accelX, "Y filtered", accelY, "Z filtered", accelZ, "X diff", avg_diff[2], "y diff", avg_diff[3], "z diff", avg_diff[4]);
//  plot.AddTimeGraph( "Nunchuk Acceleration", 500, "X filtered", accelX, "Y filtered", accelY, "Z filtered", accelZ);
//  plot.AddTimeGraph( "Nunchuk Acceleration diff", 500, "X diff", avg_diff[2], "y diff", avg_diff[3], "z diff", avg_diff[4]);

//  plot.AddTimeGraph( "Nunchuk Joy + diff", 500, "X", analogX, "Y", analogY, "X diff", avg_diff[0], "Y diff", avg_diff[1] );
//  plot.AddTimeGraph( "Nunchuk Joy", 500, "X", analogX, "Y", analogY);
//  plot.AddTimeGraph( "Nunchuk Joy diff", 500, "X diff", avg_diff[0], "Y diff", avg_diff[1] );

//  plot.AddTimeGraph( "Nunchuk Buttons", 500, "Button C", cButton, "Button Z", zButton );
//  plot.AddTimeGraph( "Nunchuk Buttons Sum", 500, "Sum C", avg_sum[6], "Sum Z", avg_sum[5] );

#endif


  ArduinoNunchuk::update();
}


bool ArduinoNunchuk::reInit()
{

    /* Power Cycle Nunchuk */
  #ifdef NUNCHUK_GNDPIN
    pinMode(NUNCHUK_GNDPIN,INPUT);
  #endif
  #ifdef NUNCHUK_VCCPIN
    pinMode(NUNCHUK_VCCPIN,INPUT);
  #endif

  delay(100);

  #ifdef NUNCHUK_GNDPIN
    pinMode(NUNCHUK_GNDPIN,OUTPUT);
    digitalWrite(NUNCHUK_GNDPIN,LOW);
  #endif
  #ifdef NUNCHUK_VCCPIN
    pinMode(NUNCHUK_VCCPIN,OUTPUT);
    digitalWrite(NUNCHUK_VCCPIN,HIGH);
  #endif

  delay(100);

  if(!ArduinoNunchuk::_sendByte(0x55, 0xF0)) return false;
  return ArduinoNunchuk::_sendByte(0x00, 0xFB);
}

int ArduinoNunchuk::update() {
  // Send sensor value request
  if(ArduinoNunchuk::_sendByte(0x00, 0x00)) return NUNCHUK_ERR_SEND;
  delay(1); // TODO: Is this needed? Maybe to give enough time to sample data?


  // request 6 bytes
  if(Wire.requestFrom(ADDRESS, 6) != 6) return NUNCHUK_ERR_COUNT;


  // read 6 bytes
  unsigned char values[6];
  int count   = 0;
  int countFF = 0;
  int count00 = 0;

  while(Wire.available()) {
    values[count] = Wire.read();
    if(values[count] == 0xFF) countFF++;
    if(values[count] == 0x00) count00++;
    count++;
  }
  if(count!=6)   return NUNCHUK_ERR_COUNT;
  if(countFF==6) return NUNCHUK_ERR_NOINIT; // if all Bytes are FF, the Nunchuk needs to be initialised probably. Errors indicate communication problems.
  if(count00==6) return NUNCHUK_ERR_ZERO;   // if all Bytes are 00.


// process data

  // remove oldest value from sum
  for(int i = 0; i<7; i++) {
    avg_sum[i] -= avg_history[i][avg_ptr];
  }

  // get latest values
  avg_history[0][avg_ptr] = values[0];                                         // analogX
  avg_history[1][avg_ptr] = values[1];                                         // analogY
  avg_history[2][avg_ptr] = ((values[2] << 2) | ((values[5] >> 2) & 3))-511;   // accelX
  avg_history[3][avg_ptr] = ((values[3] << 2) | ((values[5] >> 4) & 3))-511;   // accelY
  avg_history[4][avg_ptr] = ((values[4] << 2) | ((values[5] >> 6) & 3))-511;   // accelZ
  avg_history[5][avg_ptr] = !((values[5] >> 0) & 1);                           // zButton
  avg_history[6][avg_ptr] = !((values[5] >> 1) & 1);                           // cButton

  // add latest value to sum
  for(int i = 0; i<7; i++) {
    avg_sum[i] += avg_history[i][avg_ptr];
  }

  // check if new values are valid
  int deviationCount = 0;

  #ifndef DEBUG_PLOTTER
    int avg_diff[5];
  #endif

  for(int i = 0; i<5; i++) {
    avg_diff[i] = avg_history[i][avg_ptr] - ( avg_sum[i] / NUNCHUK_HISTORY );
//    if(avg_diff[i] > NUNCHUK_HIST_ANALOGTHRESH ) avg_diff[i] = NUNCHUK_HIST_ANALOGTHRESH ;
//    else if(avg_diff[i] < (-NUNCHUK_HIST_ANALOGTHRESH ) ) avg_diff[i] = -NUNCHUK_HIST_ANALOGTHRESH ;
  }

  if( abs( avg_diff[0] ) < NUNCHUK_HIST_ANALOGTHRESH ) {
    ArduinoNunchuk::analogX = avg_history[0][avg_ptr];
  } else {
    deviationCount++;
  }

  if( abs( avg_diff[1] ) < NUNCHUK_HIST_ANALOGTHRESH ) {
    ArduinoNunchuk::analogY = avg_history[1][avg_ptr];
  } else {
    deviationCount++;
  }

  if( abs( avg_diff[2] ) < NUNCHUK_HIST_ACCELTHRESH ) {
    ArduinoNunchuk::accelX = avg_history[2][avg_ptr];
  } else {
    deviationCount++;
  }

  if( abs( avg_diff[3] ) < NUNCHUK_HIST_ACCELTHRESH ) {
    ArduinoNunchuk::accelY = avg_history[3][avg_ptr];
  } else {
    deviationCount++;
  }

  if( abs( avg_diff[4] ) < NUNCHUK_HIST_ACCELTHRESH ) {
    ArduinoNunchuk::accelZ = avg_history[4][avg_ptr];
  } else {
    deviationCount++;
  }

  if( abs( ( avg_history[5][avg_ptr] * NUNCHUK_HISTORY ) - avg_sum[5] ) <= NUNCHUK_HIST_BUTTONTHRESH ) {
    zButton_last = zButton;
    ArduinoNunchuk::zButton = avg_history[5][avg_ptr];
  } else {
    deviationCount++;
  }

  if( abs( ( avg_history[6][avg_ptr] * NUNCHUK_HISTORY ) - avg_sum[6] ) <= NUNCHUK_HIST_BUTTONTHRESH ) {
    cButton_last = cButton;
    ArduinoNunchuk::cButton = avg_history[6][avg_ptr];
  } else {
    deviationCount++;
  }

  avg_ptr++;
  if(NUNCHUK_HISTORY == avg_ptr) avg_ptr = 0;

  #ifdef DEBUG_PLOTTER
    plot.Plot();
  #endif

  if(deviationCount > 0) return NUNCHUK_ERR_DEV1 + deviationCount - 1;


  return NUNCHUK_ERR_NOERR;
}

void ArduinoNunchuk::slowReset(int &variable, int goal, int step) {
  if      ((variable - goal) > step) variable -= step;
  else if ((goal - variable) > step) variable += step;
  else                               variable  = goal;
}

int ArduinoNunchuk::update(volatile double &pwm, volatile double &steer) {
  int error = update();
  if(error != NUNCHUK_ERR_NOERR && error != NUNCHUK_ERR_DEV1 && error != NUNCHUK_ERR_DEV1 ) return error;

  if(cButton && !zButton) {
  /* acceleration control mode when cButton is pressed */
    if(cButton_last != cButton) {
      /* use current position as zero */
      accelX_start = accelX;
      accelY_start = accelY;
      accelZ_start = accelZ;
      pitch_zero   = pitchangle();
      yaw_zero     = yawangle();
      roll_zero    = rollangle();
    }
    double newSteer = scaleAngle(rollangle()  - roll_zero , 1000.0 / NUNCHUK_ACCEL_STEER_ANGLE);
    double newPwm = scaleAngle(pitchangle() - pitch_zero, 1000.0 / NUNCHUK_ACCEL_SPEED_ANGLE);

    newSteer = steer + limit(-70.0, newSteer - steer, 70.0);
    newPwm = pwm + limit(-70.0, newPwm - pwm, 70.0);

    steer = (steer * 0.5) + (0.5 * newSteer);
    pwm = (pwm * 0.5) + (0.5 * newPwm);
  } else if (cButton && zButton) {
  /* Joystick calibration mode when both buttons are pressed */
    if((zButton_last != zButton) || (cButton_last != cButton)) { // do calibration
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
    steer = 0.0;
    pwm = 0.0;
  } else if(!cButton && zButton && !zButton_last && (rollangle() > 90.0 || rollangle() < -90.0) ) {

  } else if(cButton_last && !cButton) {
    // When Button is released, set to very low value, other than 0.0. TODO: Just a hack, for paddelec + nunchuck
    steer = 1.0;
    pwm = 1.0;

  } else {
    if(!cButton && zButton) {
      sendBuzzer.buzzerFreq = 4;
      sendBuzzer.buzzerPattern = 0;
      sendBuzzer.buzzerLen = 10;
    }

  /* use Joystick as Input */
    // check if calib is plausible
    if(analogX_min  < NUNCHUK_JOYSTICK_THRESHOLD_LOW  && analogX_max  > NUNCHUK_JOYSTICK_THRESHOLD_HIGH
    && analogY_min  < NUNCHUK_JOYSTICK_THRESHOLD_LOW  && analogY_max  > NUNCHUK_JOYSTICK_THRESHOLD_HIGH
    && analogX_zero > NUNCHUK_JOYSTICK_THRESHOLD_LOW  && analogX_zero < NUNCHUK_JOYSTICK_THRESHOLD_HIGH
    && analogY_zero > NUNCHUK_JOYSTICK_THRESHOLD_LOW  && analogY_zero < NUNCHUK_JOYSTICK_THRESHOLD_HIGH) {
      if((analogX - analogX_zero)>0) steer = (analogX - analogX_zero) * 1000.0/(analogX_max - analogX_zero) * NUNCHUK_JOYSTICK_STEER_MULT;
      else                           steer = (analogX - analogX_zero) * 1000.0/(analogX_zero - analogX_min) * NUNCHUK_JOYSTICK_STEER_MULT;

      if((analogY - analogY_zero)>0) pwm = (analogY - analogY_zero) * 1000.0/(analogY_max - analogY_zero) * NUNCHUK_JOYSTICK_SPEED_MULT;
      else                           pwm = (analogY - analogY_zero) * 1000.0/(analogY_zero - analogY_min) * NUNCHUK_JOYSTICK_SPEED_MULT;

    } else {
      steer = 0.0;
      pwm = 0.0;
    }
  }
  return NUNCHUK_ERR_NOERR;
}

uint8_t ArduinoNunchuk::_sendByte(byte data, byte location)
{
  Wire.beginTransmission(ADDRESS);

  if(Wire.write(location) != 1) return 9;
  if(Wire.write(data) != 1) return 9;

  return Wire.endTransmission();
}

void ArduinoNunchuk::debug(Stream &port)
{
  port.print("N ");
  port.printf("%4i %4i | ", analogX, analogY);
  port.printf("%4i %4i %4i | ", accelX, accelY, accelZ);
  port.printf("%2i %2i  \n", zButton, cButton);
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