#include "output.h"
#include <Arduino.h>
#include "main.h"
#include "config.h"
#include <crc.h>
#include "serialbridge.h"


#include <HoverboardAPI.h>


volatile BUZZER sendBuzzer = {
    .buzzerFreq = 0,
    .buzzerPattern = 0,
    .buzzerLen = 0,
};

#ifdef OUTPUT_PROTOCOL
  int serialWrapper(unsigned char *data, int len) {
      return (int) COM[MOTOR_COM]->write(data,len);
  }

  HoverboardAPI hoverboard = HoverboardAPI(serialWrapper);
#endif

#if defined(OUTPUT_ESPNOW) || defined(INPUT_ESPNOW)
  #include "ESP32_espnow_MasterSlave.h"
  #include "input.h"
  int scanCounter = 0;
#endif

#ifdef WIFI
  #include <WiFi.h>
  extern WiFiClient TCPClient[NUM_COM][MAX_NMEA_CLIENTS];
#endif


double limit(double min, double value, double max) {
  if(value<min) value = min;
  if(value>max) value = max;
  return value;
}

#ifdef OUTPUT_PROTOCOL

void processHalldata() {
  motor.measured.actualSpeed_kmh = hoverboard.getSpeed_kmh();
  motor.measured.actualSteer_kmh = hoverboard.getSteer_kmh();

#ifdef INPUT_ESPNOW
    if(espnowTimeout < 10) {
        if (SlaveCnt > 0) { // check if slave channel is defined
            // `slave` is defined
            sendData((const void *) &motor.measured, sizeof(motor.measured));
        } else {
            ScanForSlave();
            if (SlaveCnt > 0) { // check if slave channel is defined
            // `slave` is defined
            // Add slave as peer if it has not been added already
            manageSlave();
            // pair success or already paired
            }
        }
    }
#endif
}
#endif

void setupOutput() {

  #ifdef OUTPUT_ESPNOW
    setupEspNow();
  #endif

  #ifdef OUTPUT_PROTOCOL
    hoverboard.setPostread(0x02, processHalldata);
  #endif

}

void motorCommunication( void * pvparameters) {
#ifdef MULTITASKING
//  int taskno = (int)pvparameters;
  while(1) {
#endif //MULTITASKING

#ifdef OUTPUT_ESPNOW
  if (SlaveCnt > 0) { // check if slave channel is defined
    // `slave` is defined
    sendData((const void *) &motor.setpoint, sizeof(motor.setpoint));

    if( (sendBuzzer.buzzerFreq != 0) || (sendBuzzer.buzzerLen != 0) || (sendBuzzer.buzzerPattern != 0) ) {
      sendData((const void *) &sendBuzzer, sizeof(sendBuzzer));

      sendBuzzer.buzzerFreq = 0;
      sendBuzzer.buzzerLen = 0;
      sendBuzzer.buzzerPattern = 0;
    }

  } else if(scanCounter == 0) {
    ScanForSlave();
    if (SlaveCnt > 0) { // check if slave channel is defined
      // `slave` is defined
      // Add slave as peer if it has not been added already
      manageSlave();
      // pair success or already paired
    }
    scanCounter = 10000 / MOTORINPUT_PERIOD; // Scan only every 10 s
  } else {
    scanCounter--;
  }
  extern volatile int sendTimeout;
  extern volatile bool sendReady;
  if(sendTimeout > 100) sendReady = true;
#endif

#ifdef OUTPUT_PROTOCOL
    updateSpeed();
    hoverboard.sendSpeed(motor.setpoint.pwm, motor.setpoint.steer);

    hoverboard.protocolTick();

    /* Read and Process Incoming data */
    while(COM[MOTOR_COM]->available()) {
      hoverboard.protocolPush( COM[MOTOR_COM]->read() );
    }

    hoverboard.requestHall();
    hoverboard.protocolTick();


    /* Read and Process Incoming data */
    while(COM[MOTOR_COM]->available()) {
      hoverboard.protocolPush( COM[MOTOR_COM]->read() );
    }

    /* Send Buzzer Data */
    // TODO: Find better way to find out when to send data. This way edge case 0, 0, 0 can not be sent.
    if( (sendBuzzer.buzzerFreq != 0) || (sendBuzzer.buzzerLen != 0) || (sendBuzzer.buzzerPattern != 0) ) {
      hoverboard.sendBuzzer(sendBuzzer.buzzerFreq, sendBuzzer.buzzerLen, sendBuzzer.buzzerPattern);


      sendBuzzer.buzzerFreq = 0;
      sendBuzzer.buzzerLen = 0;
      sendBuzzer.buzzerPattern = 0;
    }



#endif
#ifdef OUTPUT_BINARY
    /* cast & limit values to a valid range */
    int16_t steer = (int16_t) limit(-1000.0, motor.setpoint.steer, 1000.0);
    int16_t pwm   = (int16_t) limit(-1000.0, motor.setpoint.pwm,   1000.0);


    /* Send motor pwm values to motor control unit */
    COM[MOTOR_COM]->write((uint8_t *) &steer, sizeof(steer));
    COM[MOTOR_COM]->write((uint8_t *) &pwm,   sizeof(pwm));

  #ifdef OUTPUT_BINARY_CRC
    /* calc and send checksum */
    uint32_t crc = 0;
    crc32((const void *)&steer, sizeof(steer), &crc);
    crc32((const void *)&pwm,   sizeof(pwm),   &crc);

    COM[MOTOR_COM]->write((uint8_t *) &crc,   sizeof(crc));
  #endif

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
    if(debug) COM[DEBUG_COM]->printf("\nU: ");
//    if(debug) COM[DEBUG_COM]->printf("%6i %6i %11u ", pwm, steer, crc);
    if(debug) COM[DEBUG_COM]->printf("%6i %6i ", pwm, steer);
    if(debug) COM[DEBUG_COM]->printf("%7.2f %7.2f ", motor.measured.actualSpeed_kmh, motor.measured.actualSteer_kmh);
#endif
#ifdef MULTITASKING
    delay(MOTORINPUT_PERIOD);
  }
#endif //MULTITASKING
}

/*
* Dummy function since no speed feedback from Motor control is implemented right now.
* For now, we just use pwm, some conversion factor and low pass filter as a model.
* Values are in m/h
*/
#define SPEED_PWM_CONVERSION_FACTOR  0.2   // Assume 100% PWM = 1000 = Full Speed = 20km/h = 20000 m/h. Therefore 20000 / 1000 = 20
#define SPEED_FILTER                 0.015  // Low pass Filter Value. 1 means no filter at all, 0 no value update.
void updateSpeed() {

#if !defined(OUTPUT_PROTOCOL) && !defined (OUTPUT_ESPNOW)
  motor.measured.actualSpeed_kmh = motor.measured.actualSpeed_kmh * (1.0 - (SPEED_FILTER * deltaMillis)) + motor.setpoint.pwm   * (SPEED_FILTER * deltaMillis) * SPEED_PWM_CONVERSION_FACTOR;
  motor.measured.actualSteer_kmh = motor.measured.actualSteer_kmh * (1.0 - (SPEED_FILTER * deltaMillis)) + motor.setpoint.steer * (SPEED_FILTER * deltaMillis) * SPEED_PWM_CONVERSION_FACTOR;
#endif


#ifdef INPUT_ESPNOW
  if(espnowTimeout > 100) {
    if (SlaveCnt > 0) { // check if slave channel is defined
      // `slave` is defined
      sendData((const void *) &motor.measured, sizeof(motor.measured));
    } else if(scanCounter == 0) {
      ScanForSlave();
      if (SlaveCnt > 0) { // check if slave channel is defined
        // `slave` is defined
        // Add slave as peer if it has not been added already
        manageSlave();
        // pair success or already paired
      }
      scanCounter = 10000 / MOTORINPUT_PERIOD; // Scan only every 10 s
    } else {
      scanCounter--;
    }
  }
  extern volatile int sendTimeout;
  extern volatile bool sendReady;
  if(sendTimeout > 100) sendReady = true;
#endif

}
