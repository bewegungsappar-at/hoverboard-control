#include <Arduino.h>
#include "config.h"
#include <crc.h>
#include "serialbridge.h"

#ifdef PADDELEC
  #include <Paddelec.h>
  Paddelec paddelec = Paddelec();
#endif // PADDELEC

#ifdef NUNCHUCK
  #include <ArduinoNunchuk.h>
  ArduinoNunchuk nunchuk = ArduinoNunchuk();
#endif // NUNCHUCK

bool debug = false;

struct motorControl 
{
  double steer;
  double pwm;     // called "speed" in hoverboard firmware, but is only pwm duty cycle in promille
                  // Values from -1000 to 1000. Negative values represent driving backwards.
  double actualSpeed_kmh;  // motor speed in m/h
  double actualSteer_kmh;  // motor steer
};

motorControl motor = {0.0, 0.0, 0.0, 0.0};
uint32_t nextMillisMotorInput = 0;      // virtual timer for motor update
uint32_t deltaMillis;

void setup() {

#ifdef DEBUG
  debug = true;
#endif

  setupSerial();
  setupWifi();
  setupSerialbridge();
    
#ifdef OTA_HANDLER  
  setupOTA();
#endif 

#ifdef NUNCHUCK
  nunchuk.init();
#endif

  /* Keep this at the end of setup */
  nextMillisMotorInput = millis();
}


double limit(double min, double value, double max) 
{
  if(value<min) value = min;
  if(value>max) value = max;
  return value;
}


/* 
* Dummy function since no speed feedback from Motor control is implemented right now.
* For now, we just use pwm, some conversion factor and low pass filter as a model.
* Values are in m/h 
*/ 
#define SPEED_PWM_CONVERSION_FACTOR  0.2   // Assume 100% PWM = 1000 = Full Speed = 20km/h = 20000 m/h. Therefore 20000 / 1000 = 20
#define SPEED_FILTER                 0.015  // Low pass Filter Value. 1 means no filter at all, 0 no value update.
void updateSpeed()
{
  motor.actualSpeed_kmh = motor.actualSpeed_kmh * (1.0 - (SPEED_FILTER * deltaMillis)) + motor.pwm   * (SPEED_FILTER * deltaMillis) * SPEED_PWM_CONVERSION_FACTOR;
  motor.actualSteer_kmh = motor.actualSteer_kmh * (1.0 - (SPEED_FILTER * deltaMillis)) + motor.steer * (SPEED_FILTER * deltaMillis) * SPEED_PWM_CONVERSION_FACTOR;
}

void loop() 
{  
#ifdef OTA_HANDLER  
  ArduinoOTA.handle();
#endif // OTA_HANDLER

  bridge();

  deltaMillis = millis() - nextMillisMotorInput;
  if(deltaMillis >= 0)
  {
    nextMillisMotorInput += MOTORINPUT_PERIOD;

#ifdef PADDELEC
    paddelec.update(motor.pwm, motor.steer, motor.actualSpeed_kmh, motor.actualSteer_kmh, deltaMillis);
    if(debug)
    {
      paddelec.debug(*COM[DEBUG_COM]);
      for(byte cln = 0; cln < MAX_NMEA_CLIENTS; cln++)
        if(TCPClient[1][cln]) paddelec.debug(TCPClient[1][cln]); 
    }
#endif // PADDELEC

#if defined(NUNCHUCK) && defined(PADDELEC)
    if(paddelec.gametrak1.r < 500 || paddelec.gametrak2.r < 500) // switch to nunchuck control when paddle is not used
    {
#endif
#if defined(NUNCHUCK)
      int nunchuckError = nunchuk.update(motor.pwm, motor.steer);
      nunchuk.debug(*COM[DEBUG_COM]);
      if(nunchuckError >= 1000) 
      {
        nunchuk.reInit();
        if(debug) COM[DEBUG_COM]->printf("Reinit Nunchuck %4i ", nunchuckError);
      } else if(nunchuckError >= 100) 
      {
        nunchuk.reInit();
        if(debug) COM[DEBUG_COM]->printf("I2C Problems %4i ", nunchuckError);
      } else if(nunchuckError > 0)
        if(debug) COM[DEBUG_COM]->printf("Nunchuck Comm Problems %4i ", nunchuckError);
#endif
#if defined(NUNCHUCK) && defined(PADDELEC)
    }
#endif

#if defined(NUNCHUCK) && !defined(PADDELEC) 

#endif // NUNCHUCK

    /* limit values to a valid range */
    motor.pwm   = limit(-1000.0, motor.pwm,   1000.0);  
    motor.steer = limit(-1000.0, motor.steer, 1000.0);  

    int16_t steer = (int16_t) motor.steer;
    int16_t pwm   = (int16_t) motor.pwm;
    
    /* calc checksum */
    uint32_t crc = 0;
    crc32((const void *)&steer, sizeof(steer), &crc); 
    crc32((const void *)&pwm,   sizeof(pwm),   &crc); 
    
    /* Send motor pwm values to motor control unit */
    COM[MOTOR_COM]->write((uint8_t *) &steer, sizeof(steer)); 
    COM[MOTOR_COM]->write((uint8_t *) &pwm,   sizeof(pwm));
    COM[MOTOR_COM]->write((uint8_t *) &crc,   sizeof(crc));

    /* refresh actual motor speed */
    updateSpeed();

    /* debug output */
    for(byte cln = 0; cln < MAX_NMEA_CLIENTS; cln++)
    {   
      if(TCPClient[1][cln]) {                    
        if(debug) TCPClient[1][cln].print(" U: ");
        if(debug) TCPClient[1][cln].printf("%8i %8i\n", pwm, steer);
      } 
    }
    if(debug) COM[DEBUG_COM]->print(" U: ");
    if(debug) COM[DEBUG_COM]->printf("%6i %6i %11u ", pwm, steer, crc);
    if(debug) COM[DEBUG_COM]->printf("%5.2f %5.2f \n", motor.actualSpeed_kmh, motor.actualSteer_kmh);
  }
  if(deltaMillis >= MOTORINPUT_PERIOD)  // check if system is too slow
  {
    if(debug) COM[DEBUG_COM]->print(" Missed Period: ");
    if(debug) COM[DEBUG_COM]->print(deltaMillis); 
    if(debug) COM[DEBUG_COM]->print("ms ");
    
    nextMillisMotorInput = millis() + MOTORINPUT_PERIOD;
  }
/* TODO
*  calculate paddle angle. if only gametrak angles are subtracted, the activation threshold depends on the distance (r)
*  independet variables for left and right wheel, recover MIXER
*  CRC checksum, maybe message counter?
*  perfect freewheeling, paddle strenght effect based on paddle angle
*  define minimum r to function
*  check phi vor valid values
*  use timer to get constant deltas
*  put everything inti functions
*/
}