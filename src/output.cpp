#include "output.h"
#include <Arduino.h>
#include "main.h"
#include "config.h"
#include <crc.h>
#include "serialbridge.h"


#include <HoverboardAPI.h>


volatile BUZZER_DATA sendBuzzer = {
    .buzzerFreq = 0,
    .buzzerPattern = 0,
    .buzzerLen = 0,
};

#ifdef OUTPUT_PROTOCOL
  int serialWrapper(unsigned char *data, int len) {
//#define DEBUG_PROTOCOL_OUTGOING_MARKUP
#ifdef DEBUG_PROTOCOL_OUTGOING_MARKUP
      for(int i = 0; i< len; i++) {
        switch (i) {
        case 0:
          Serial.printf("SOM:%01i ", data[i]);
          break;

        case 1:
          Serial.printf("CI:%03i ", data[i]);
          break;

        case 2:
          Serial.printf("len:%03i ", data[i]);
          break;

        case 3:
          Serial.printf("CMD:%c ", data[i]);
          break;

        case 4:
          if(i==len-1) {
            Serial.printf("CS:0x%02X ", data[i]);
          } else if(data[i] == hoverboardCodes::setPointPWM) {
            Serial.print("PWM       ");
          } else if(data[i] == hoverboardCodes::protocolSubscriptions) {
            Serial.print("Subscribe ");
          } else if(data[i] == hoverboardCodes::sensHall) {
            Serial.print("Hall      ");
          } else if(data[i] == hoverboardCodes::protocolCountSum) {
            Serial.print("CounterS  ");
          } else if(data[i] == hoverboardCodes::setBuzzer) {
            Serial.print("Buzzer    ");
          } else if(data[i] == hoverboardCodes::sensElectrical) {
            Serial.print("El. Meas  ");
          } else {
            Serial.printf("Code:0x%02X ", data[i]);
          }
          break;

        default:
          if(i==len-1) {
            Serial.printf("CS:0x%02X ", data[i]);
          } else {
            Serial.printf("%02X ", data[i]);
          }
          break;
        }
      }
      Serial.println();
#endif
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


void processHalldata ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type ) {
  switch (fn_type) {
    case FN_TYPE_POST_READRESPONSE:
    case FN_TYPE_POST_WRITE:
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

      break;
  }
}
#endif

//#define DEBUG_PROTOCOL_PASSTHROUGH


void setupOutput() {

  #ifdef OUTPUT_ESPNOW
    setupEspNow();
  #endif

  #ifdef OUTPUT_PROTOCOL
    hoverboard.setParamHandler(hoverboardCodes::sensHall, processHalldata);

    #ifndef DEBUG_PROTOCOL_PASSTHROUGH
      hoverboard.scheduleTransmission(hoverboardCodes::setPointPWM, -1, 30);

      // schudule a scheduling request for Hall Data:
      // 100 Messages for 30 msec = 3s. Repeat each 1000ms, indefinetly (-1)
      hoverboard.scheduleScheduling(hoverboardCodes::sensHall, 100, 30, 1000, -1);
      hoverboard.scheduleRead(hoverboardCodes::protocolCountSum, -1, 30);

    #endif
  #endif

}


void pollUART() {
  #ifdef OUTPUT_PROTOCOL
    // Read and Process Incoming data
    int i=0;
    while(COM[MOTOR_COM]->available() && i++ < 1024) { // read maximum 1024 byte at once.
      unsigned char readChar = COM[MOTOR_COM]->read();
      hoverboard.protocolPush( readChar );
      #ifdef DEBUG_PROTOCOL_PASSTHROUGH
        COM[DEBUG_COM]->write( readChar );
      #endif
    }

    #ifdef DEBUG_PROTOCOL_PASSTHROUGH
      while(COM[DEBUG_COM]->available() && i++ < 1024) { // read maximum 1024 byte at once.
        COM[MOTOR_COM]->write( COM[DEBUG_COM]->read() );
      }
    #endif
    hoverboard.protocolTick();
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

//    hoverboard.sendPWM(motor.setpoint.pwm, motor.setpoint.steer);
//    hoverboard.requestRead(hoverboardCodes::sensHall);

  #ifndef DEBUG_PROTOCOL_PASSTHROUGH
    hoverboard.requestRead(hoverboardCodes::sensElectrical);
//    hoverboard.requestRead(hoverboardCodes::sensHall);
    Serial.print("V: ");
    Serial.print(hoverboard.getBatteryVoltage());
    Serial.print(" Current0: ");
    Serial.print(hoverboard.getMotorAmpsAvg(0));
    Serial.print(" Current1: ");
    Serial.print(hoverboard.getMotorAmpsAvg(1));
    Serial.print(" Speed: ");
    Serial.print(hoverboard.getSpeed_kmh());
    Serial.print(" Steer: ");
    Serial.print(hoverboard.getSteer_kmh());
    Serial.println();

  #endif

    extern PWM_DATA PWMData;

    PWMData.pwm[0] = motor.setpoint.pwm - motor.setpoint.steer;
    PWMData.pwm[1] = motor.setpoint.pwm + motor.setpoint.steer;
//    hoverboard.scheduleRead(hoverboardCodes::protocolCountSum, -1, 30);
//    hoverboard.printStats(*COM[DEBUG_COM]);


    // Send Buzzer Data
    // TODO: Find better way to find out when to send data. This way edge case 0, 0, 0 can not be sent.
    if( (sendBuzzer.buzzerFreq != 0) || (sendBuzzer.buzzerLen != 0) || (sendBuzzer.buzzerPattern != 0) ) {
      hoverboard.sendBuzzer(sendBuzzer.buzzerFreq, sendBuzzer.buzzerPattern, sendBuzzer.buzzerLen);

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
#ifdef OUTPUT_PROTOCOL
    pollUART();
//    Serial.print(" ");
//    Serial.print(hoverboard.getTxBufferLevel());
//    Serial.print(" ");
#endif

#ifdef MULTITASKING

    unsigned long start = millis();
    while (millis() < start + MOTORINPUT_PERIOD){
      pollUART();
      delayMicroseconds(100);
    }
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
