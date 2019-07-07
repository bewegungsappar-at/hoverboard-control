#include "output.h"
#include <Arduino.h>
#include "main.h"
#include "config.h"
#include <crc.h>
#include "serialbridge.h"
#include "protocol.h"

#include <HoverboardAPI.h>

#if defined(OUTPUT_ESPNOW) || defined(INPUT_ESPNOW) || defined(OUTPUT_PROTOCOL_ESPNOW)
  #include "ESP32_espnow_MasterSlave.h"
  #include "input.h"
  #include <esp_now.h>
  int scanCounter = 0;
#endif

volatile PROTOCOL_BUZZER_DATA sendBuzzer = {
    .buzzerFreq = 0,
    .buzzerPattern = 0,
    .buzzerLen = 0,
};

PROTOCOL_PWM_DATA PWMData = {
    .pwm = {0,0},
    .speed_max_power =  600,
    .speed_min_power = -600,
    .speed_minimum_pwm = 40 // guard value, below this set to zero
};

uint8_t enableHoverboardMotors = 0;

#ifdef OUTPUT_PROTOCOL_UART
  int serialWrapper(unsigned char *data, int len) {
#ifdef DEBUG_PROTOCOL_OUTGOING_MARKUP
      for(int i = 0; i< len; i++) {
        switch (i) {
        case 0:
          COM[DEBUG_COM]->printf("SOM:%01i ", data[i]);
          break;

        case 1:
          COM[DEBUG_COM]->printf("CI:%03i ", data[i]);
          break;

        case 2:
          COM[DEBUG_COM]->printf("len:%03i ", data[i]);
          break;

        case 3:
          COM[DEBUG_COM]->printf("CMD:%c ", data[i]);
          break;

        case 4:
          if(i==len-1) {
            COM[DEBUG_COM]->printf("CS:0x%02X ", data[i]);
          } else if(data[i] == HoverboardAPI::Codes::setPointPWM) {
            COM[DEBUG_COM]->print("PWM       ");
          } else if(data[i] == HoverboardAPI::Codes::setPointPWMData) {
            COM[DEBUG_COM]->print("PWM Data  ");
          } else if(data[i] == HoverboardAPI::Codes::protocolSubscriptions) {
            COM[DEBUG_COM]->print("Subscribe ");
          } else if(data[i] == HoverboardAPI::Codes::sensHall) {
            COM[DEBUG_COM]->print("Hall      ");
          } else if(data[i] == HoverboardAPI::Codes::protocolCountSum) {
            COM[DEBUG_COM]->print("CounterS  ");
          } else if(data[i] == HoverboardAPI::Codes::setBuzzer) {
            COM[DEBUG_COM]->print("Buzzer    ");
          } else if(data[i] == HoverboardAPI::Codes::enableMotors) {
            COM[DEBUG_COM]->print("Enable    ");
          } else if(data[i] == HoverboardAPI::Codes::sensElectrical) {
            COM[DEBUG_COM]->print("El. Meas  ");
          } else {
            COM[DEBUG_COM]->printf("Code:0x%02X ", data[i]);
          }
          break;

        default:
          if(i==len-1) {
            COM[DEBUG_COM]->printf("CS:0x%02X ", data[i]);
          } else {
            COM[DEBUG_COM]->printf("%02X ", data[i]);
          }
          break;
        }
      }
      COM[DEBUG_COM]->println();
#endif
      return (int) COM[MOTOR_COM]->write(data,len);
  }

  HoverboardAPI hoverboard = HoverboardAPI(serialWrapper);
#endif

#ifdef OUTPUT_PROTOCOL_ESPNOW

  int espSendDataWrapper(unsigned char *data, int len) {
    if(espnowTimeout > 100) {
      if (SlaveCnt > 0) { // check if slave channel is defined
        // `slave` is defined
        sendData(data, (size_t) len);
        return len;
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
    return -1;
  }

  HoverboardAPI hbpEspnow = HoverboardAPI(espSendDataWrapper);

  void espReceiveDataWrapper(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    for(int i=0; i < data_len; i++) {
        hbpEspnow.protocolPush(data[i]);
    }
  }

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

#ifdef OUTPUT_PROTOCOL_UART
void processHalldata ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, unsigned char *content, int len ) {
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

void setupOutput() {

  #ifdef OUTPUT_ESPNOW
    setupEspNow();
  #endif

  #if defined(OUTPUT_PROTOCOL_UART) && !defined(DEBUG_PROTOCOL_PASSTHROUGH)

    #ifdef INPUT_TESTRUN
      // Remove PWM limits
      hoverboard.sendPWMData(0, 0, 1000, -1000, 1, PROTOCOL_SOM_ACK);

      // send enable periodically
      hoverboard.updateParamVariable(HoverboardAPI::Codes::enableMotors, &enableHoverboardMotors, sizeof(enableHoverboardMotors));
      hoverboard.scheduleTransmission(hoverboard.Codes::enableMotors, -1, 30);

      // Get Protocol statistics periodically
      hoverboard.scheduleRead(hoverboard.Codes::protocolCountSum, -1, 100);
    #else
      // Set PWM limits
      hoverboard.sendPWMData(0, 0, 400, -400, 30, PROTOCOL_SOM_ACK);

      // enable motors
      hoverboard.sendEnable(1, PROTOCOL_SOM_ACK);
    #endif

    // Set up hall data readout (=hoverboard measured speed)
    hoverboard.updateParamHandler(HoverboardAPI::Codes::sensHall, processHalldata);

    hoverboard.scheduleRead(HoverboardAPI::Codes::sensHall, -1, 30);

    // Set up electrical measurements readout
    hoverboard.scheduleRead(hoverboard.Codes::sensElectrical, -1, 100);

    // Send PWM values periodically
    hoverboard.updateParamVariable(HoverboardAPI::Codes::setPointPWM, &PWMData, sizeof(PWMData));
    hoverboard.scheduleTransmission(HoverboardAPI::Codes::setPointPWM, -1, 30);
  #endif



  #ifdef OUTPUT_PROTOCOL_ESPNOW
    esp_now_register_recv_cb(espReceiveDataWrapper);
    hbpEspnow.updateParamHandler(hoverboard.Codes::sensHall, processHalldata);
    hbpEspnow.scheduleTransmission(hoverboard.Codes::setPointPWM, -1, 30);
    hbpEspnow.scheduleRead(hoverboard.Codes::protocolCountSum, -1, 30);
  #endif

}

#ifdef OUTPUT_PROTOCOL_UART
void pollUART() {
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
}
#endif

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

#ifdef OUTPUT_PROTOCOL_UART
    updateSpeed();  // TODO will be obsolete soon

  #ifdef DEBUG_PROTOCOL_MEASUREMENTS
    COM[DEBUG_COM]->print("V: ");
    COM[DEBUG_COM]->print(hoverboard.getBatteryVoltage());
    COM[DEBUG_COM]->print(" Current0: ");
    COM[DEBUG_COM]->print(hoverboard.getMotorAmpsAvg(0));
    COM[DEBUG_COM]->print(" Current1: ");
    COM[DEBUG_COM]->print(hoverboard.getMotorAmpsAvg(1));
    COM[DEBUG_COM]->print(" Speed: ");
    COM[DEBUG_COM]->print(hoverboard.getSpeed_kmh());
    COM[DEBUG_COM]->print(" Steer: ");
    COM[DEBUG_COM]->print(hoverboard.getSteer_kmh());
    COM[DEBUG_COM]->println();
  #endif


    PWMData.pwm[0] = motor.setpoint.pwm + motor.setpoint.steer;
    PWMData.pwm[1] = motor.setpoint.pwm - motor.setpoint.steer;

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

#ifdef OUTPUT_PROTOCOL_UART
    pollUART();
    hoverboard.protocolTick();
#endif

#ifdef OUTPUT_PROTOCOL_ESPNOW
    hbpEspnow.protocolTick();
#endif

#ifdef MULTITASKING
    unsigned long start = millis();
    while (millis() < start + MOTORINPUT_PERIOD){
      #ifdef OUTPUT_PROTOCOL_UART
        pollUART();
        hoverboard.protocolTick();
      #endif

      #ifdef OUTPUT_PROTOCOL_ESPNOW
        hbpEspnow.protocolTick();
      #endif

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

  #if !defined(OUTPUT_PROTOCOL_UART) && !defined (OUTPUT_ESPNOW)
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
