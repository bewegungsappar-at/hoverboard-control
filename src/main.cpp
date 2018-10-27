#include <Arduino.h>
#include "config.h"
#include <crc.h>
#include "serialbridge.h"

TaskHandle_t TaskMainloop, TaskMotorcommunication;
void mainloop(void *pvParameters);
void motorCommunication(void *pvParameters);

#ifdef PADDELEC
  #include "Paddelec.h"
  Paddelec paddelec = Paddelec();
#endif // PADDELEC

#ifdef NUNCHUCK
  #include <ArduinoNunchuk.h>
  ArduinoNunchuk nunchuk = ArduinoNunchuk();
#endif // NUNCHUCK

#ifdef PLATOONING
  #include "Platooning.h"
  Platooning platooning = Platooning();
#endif

#ifdef WIFI

#endif

#ifdef BLE
  #include "BLE.h"
#endif

#ifdef IMU
  #include <IMU.h>
  Imu imu = Imu();
#endif

bool debug = false;

struct motorControl {
  double steer;
  double pwm;     // called "speed" in hoverboard firmware, but is only pwm duty cycle in promille
                  // Values from -1000 to 1000. Negative values represent driving backwards.
  double actualSpeed_kmh;  // motor speed in m/h
  double actualSteer_kmh;  // motor steer
};

motorControl motor = {0.0, 0.0, 0.0, 0.0};
uint32_t nextMillisMotorInput = 0;      // virtual timer for motor update
int32_t deltaMillis;
int errorCount=0;

void setup() {

#ifdef DEBUG
  debug = true;
#endif

  setupSerial();

#ifdef WIFI
  setupWifi();
  setupSerialbridge();
#endif
    
#ifdef OTA_HANDLER  
  setupOTA();
#endif 

#ifdef NUNCHUCK
  nunchuk.init();
#endif

#ifdef BLE
  setupBLE();
#endif

#ifdef IMU
  imu.init();
#endif

  /* Keep this at the end of setup */
  nextMillisMotorInput = millis();

  xTaskCreatePinnedToCore(
    mainloop,                 /* Task function. */
    "Main_loop",              /* name of task. */
    4000,                     /* Stack size of task */
    (void *)1,                /* parameter of the task */
    1,                        /* priority of the task */
    &TaskMainloop,            /* Task handle to keep track of created task */
    0);                       /* Core (0 is used by ESP32 connectivity) */

  xTaskCreatePinnedToCore(
    motorCommunication,       /* Task function. */
    "Motor_Comm",             /* name of task. */
    4000,                     /* Stack size of task */
    (void *)1,                /* parameter of the task */
    1,                        /* priority of the task */
    &TaskMotorcommunication,  /* Task handle to keep track of created task */
    0);                       /* Core (0 is used by ESP32 connectivity) */
}


double limit(double min, double value, double max) {
  if(value<min) value = min;
  if(value>max) value = max;
  return value;
}

void slowReset(double &variable, double goal, double step) {
  if      ((variable - goal) > step) variable -= step;
  else if ((goal - variable) > step) variable += step;
  else                               variable  = goal;
}

/* 
* Dummy function since no speed feedback from Motor control is implemented right now.
* For now, we just use pwm, some conversion factor and low pass filter as a model.
* Values are in m/h 
*/ 
#define SPEED_PWM_CONVERSION_FACTOR  0.2   // Assume 100% PWM = 1000 = Full Speed = 20km/h = 20000 m/h. Therefore 20000 / 1000 = 20
#define SPEED_FILTER                 0.015  // Low pass Filter Value. 1 means no filter at all, 0 no value update.
void updateSpeed() {
  motor.actualSpeed_kmh = motor.actualSpeed_kmh * (1.0 - (SPEED_FILTER * deltaMillis)) + motor.pwm   * (SPEED_FILTER * deltaMillis) * SPEED_PWM_CONVERSION_FACTOR;
  motor.actualSteer_kmh = motor.actualSteer_kmh * (1.0 - (SPEED_FILTER * deltaMillis)) + motor.steer * (SPEED_FILTER * deltaMillis) * SPEED_PWM_CONVERSION_FACTOR;
}

void loop() {  
  // nope, do nothing here
  vTaskDelay(portMAX_DELAY); // wait as much as posible ...
}

void mainloop( void *pvparameters ) {
  int taskno = (int)pvparameters;
  while(1) {
  #ifdef OTA_HANDLER  
    ota();
  #endif // OTA_HANDLER

  #ifdef WIFI
    bridge();
  #endif

    deltaMillis = millis() - nextMillisMotorInput;
    if(deltaMillis >= 0) {
      nextMillisMotorInput += MOTORINPUT_PERIOD;

  #ifdef PADDELEC
      paddelec.update(motor.pwm, motor.steer, motor.actualSpeed_kmh, motor.actualSteer_kmh, deltaMillis);
      if(debug) {
        paddelec.debug(*COM[DEBUG_COM]);
        for(byte cln = 0; cln < MAX_NMEA_CLIENTS; cln++)
          if(TCPClient[1][cln]) paddelec.debug(TCPClient[1][cln]); 
      }
  #endif // PADDELEC

  #ifdef BLE
  //  loopBLE();
  motor.pwm = ble_pitch;
  motor.steer = ble_roll;
  slowReset(ble_pitch, 0.0, 0.1);
  slowReset(ble_roll, 0.0, 0.1);
  #endif

  #ifdef IMU
  //  imu.loopIMU();
    imu.update(motor.pwm, motor.steer);
    imu.debug(*COM[DEBUG_COM]);
  #endif

  #if defined(NUNCHUCK) && defined(PADDELEC)
      if(paddelec.gametrak1.r < 500 || paddelec.gametrak2.r < 500) { // switch to nunchuck control when paddle is not used
  #endif
  #if defined(NUNCHUCK)
        int nunchuckError = nunchuk.update(motor.pwm, motor.steer);
        ++errorCount;
        nunchuk.debug(*COM[DEBUG_COM]);
        if(nunchuckError >= 1000) {
          if(debug) COM[DEBUG_COM]->printf("Reinit Nunchuck %4i ", nunchuckError);
        } else if(nunchuckError >= 100) {
          if(debug) COM[DEBUG_COM]->printf("I2C Problems %4i ", nunchuckError);
        } else if(nunchuckError > 0) {
          if(debug) COM[DEBUG_COM]->printf("Nunchuck Comm Problems %4i ", nunchuckError);
        } else  {
          errorCount = 0;
        }

        if(errorCount>=5) {
          nunchuk.reInit();
          errorCount = 0;
        }
  #endif
  #if defined(NUNCHUCK) && defined(PADDELEC)
      }
  #endif

  #ifdef PLATOONING
      platooning.update(motor.pwm, motor.steer);
      if(debug) {
        platooning.debug(*COM[DEBUG_COM]);
    #ifdef WIFI
        for(byte cln = 0; cln < MAX_NMEA_CLIENTS; cln++)
          if(TCPClient[1][cln]) platooning.debug(TCPClient[1][cln]); 
    #endif
      }
  #endif // PLATOONING
    } else if(deltaMillis >= MOTORINPUT_PERIOD) { // check if system is too slow
      if(debug) COM[DEBUG_COM]->print(" Missed Period: ");
      if(debug) COM[DEBUG_COM]->print(deltaMillis); 
      if(debug) COM[DEBUG_COM]->print("ms ");
      
      nextMillisMotorInput = millis() + MOTORINPUT_PERIOD;
    }
    vTaskDelay(100);
  }
}

void motorCommunication( void * pvparameters) {
  int taskno = (int)pvparameters;
  while(1) {
    /* cast & limit values to a valid range */
    int16_t steer = (int16_t) limit(-1000.0, motor.steer, 1000.0);
    int16_t pwm   = (int16_t) limit(-1000.0, motor.pwm,   1000.0);
    
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
    #ifdef WIFI
    for(byte cln = 0; cln < MAX_NMEA_CLIENTS; cln++) {   
      if(TCPClient[1][cln]) {                    
        if(debug) TCPClient[1][cln].print(" U: ");
        if(debug) TCPClient[1][cln].printf("%8i %8i\n", pwm, steer);
      } 
    }
    #endif
    if(debug) COM[DEBUG_COM]->print(" U: ");
    if(debug) COM[DEBUG_COM]->printf("%6i %6i %11u ", pwm, steer, crc);
    if(debug) COM[DEBUG_COM]->printf("%5.2f %5.2f \n", motor.actualSpeed_kmh, motor.actualSteer_kmh);

    delay(MOTORINPUT_PERIOD);             
  }
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