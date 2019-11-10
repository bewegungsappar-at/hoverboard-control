#include "communication.h"
#include <Arduino.h>
#include "main.h"
#include "config.h"
#include "serialbridge.h"
#include "protocol.h"

#include <HoverboardAPI.h>

#if defined(OUTPUT_ESPNOW) || defined(INPUT_ESPNOW)
  #include "ESP32_espnow_MasterSlave.h"
  #include "input.h"
  #include <esp_now.h>
#endif

///////////////////////////////////////////////////////////
// Global Variables
///////////////////////////////////////////////////////////

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

#if defined(OUTPUT_ESPNOW) || defined(INPUT_ESPNOW)
  int scanCounter = 0;
#endif

#ifdef OUTPUT_PROTOCOL_UART
  int serialWriteWrapper(unsigned char *data, int len);
  HoverboardAPI hbpOut = HoverboardAPI(serialWriteWrapper);
#endif

#ifdef INPUT_ESPNOW
  int espSendDataWrapper(unsigned char *data, int len);
  HoverboardAPI hbpIn = HoverboardAPI(espSendDataWrapper);
#endif

#ifdef OUTPUT_ESPNOW
  int espSendDataWrapper(unsigned char *data, int len);
  HoverboardAPI hbpOut = HoverboardAPI(espSendDataWrapper);
#endif

///////////////////////////////////////////////////////////
// Support Functions
///////////////////////////////////////////////////////////

void protocolMarkupOutgoing(unsigned char *data, int len) {
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
}


#ifdef OUTPUT_PROTOCOL_UART
int serialWriteWrapper(unsigned char *data, int len) {

  #ifdef DEBUG_PROTOCOL_OUTGOING_MARKUP
  protocolMarkupOutgoing(data, len);
  #endif

  return (int) COM[MOTOR_COM]->write(data,len);
}
#endif

#if defined(INPUT_ESPNOW) || defined(OUTPUT_ESPNOW)
int espSendDataWrapper(unsigned char *data, int len) {

  #ifdef DEBUG_PROTOCOL_OUTGOING_MARKUP
  protocolMarkupOutgoing(data, len);
  #endif

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
  return -1;
}

void espReceiveDataWrapper(const uint8_t *mac_addr, const uint8_t *data, int data_len) {

  // Print debug information
  #ifdef debugESPNOW
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    if(debug_espnow) COM[DEBUG_COM]->print("\t\tLast Packet Recv from: "); if(debug_espnow) COM[DEBUG_COM]->println(macStr);
    if(debug_espnow) COM[DEBUG_COM]->print("\t\tLast Packet Recv Data: "); if(debug_espnow) COM[DEBUG_COM]->println((char *)data);
    if(debug_espnow) COM[DEBUG_COM]->println("");
  #endif

  /* validate MAC Address
  * In ESPnow a different MAC Adress is used to send or receive packets.
  * Fortunately, the MAC Adresses are only one bit apart.
  */
  int foundSlave = 0;
  extern esp_now_peer_info_t slaves[1];

  for(int i = 0; i< SlaveCnt; i++) {
    if( slaves[i].peer_addr[0] == mac_addr[0] &&
        slaves[i].peer_addr[1] == mac_addr[1] &&
        slaves[i].peer_addr[2] == mac_addr[2] &&
        slaves[i].peer_addr[3] == mac_addr[3] &&
        slaves[i].peer_addr[4] == mac_addr[4] &&
        slaves[i].peer_addr[5] == mac_addr[5] )
    {
      foundSlave++;
    }
  }

  if(foundSlave == 0) return;


  // Stop SSID Broadcast as soon as Package was received
  extern int hideAP;
  extern void configDeviceAP();
  if(!hideAP) {
    hideAP = 1;
    configDeviceAP();
  }

  // Pass data to protocol
  for(int i=0; i < data_len; i++) {
    #if defined(OUTPUT_ESPNOW)

      hbpOut.protocolPush(data[i]);
    #elif defined(INPUT_ESPNOW)
      hbpIn.protocolPush(data[i]);
    #endif
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

#if defined(OUTPUT_ESPNOW)
void processHalldata ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, unsigned char *content, int len ) {
  switch (fn_type) {
    case FN_TYPE_POST_READRESPONSE:
    case FN_TYPE_POST_WRITE:
      motor.measured.actualSpeed_kmh = hbpOut.getSpeed_kmh();
      motor.measured.actualSteer_kmh = hbpOut.getSteer_kmh();

      break;
  }
}
#endif

#if defined(OUTPUT_PROTOCOL_UART)
void processHalldata ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, unsigned char *content, int len ) {
  switch (fn_type) {
    case FN_TYPE_POST_READRESPONSE:
    case FN_TYPE_POST_WRITE:
      motor.measured.actualSpeed_kmh = hbpOut.getSpeed_kmh();
      motor.measured.actualSteer_kmh = hbpOut.getSteer_kmh();

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

#ifdef OUTPUT_PROTOCOL_UART
void pollUART() {
  // Read and Process Incoming data
  int i=0;
  while(COM[MOTOR_COM]->available() && i++ < 1024) { // read maximum 1024 byte at once.
    unsigned char readChar = COM[MOTOR_COM]->read();
    hbpOut.protocolPush( readChar );
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


///////////////////////////////////////////////////////////
// Communication Init
///////////////////////////////////////////////////////////

void setupCommunication() {

  #if defined(INPUT_ESPNOW) || defined(OUTPUT_ESPNOW)
    setupEspNow();
  #endif

  #if defined(OUTPUT_PROTOCOL_UART) && !defined(DEBUG_PROTOCOL_PASSTHROUGH)

    #ifdef INPUT_TESTRUN
      // Remove PWM limits
      hbpOut.sendPWMData(0, 0, 1000, -1000, 1, PROTOCOL_SOM_ACK);

      // send enable periodically
      hbpOut.updateParamVariable(HoverboardAPI::Codes::enableMotors, &enableHoverboardMotors, sizeof(enableHoverboardMotors));
      hbpOut.scheduleTransmission(HoverboardAPI::Codes::enableMotors, -1, 30);

      // Get Protocol statistics periodically
      hbpOut.scheduleRead(HoverboardAPI::Codes::protocolCountSum, -1, 100);
    #else
      // Set PWM limits
      hbpOut.sendPWMData(0, 0, 400, -400, 30, PROTOCOL_SOM_ACK);

      // enable motors
      hbpOut.sendEnable(1, PROTOCOL_SOM_ACK);
    #endif

    // Set up hall data readout (=hoverboard measured speed)
    hbpOut.updateParamHandler(HoverboardAPI::Codes::sensHall, processHalldata);

    hbpOut.scheduleRead(HoverboardAPI::Codes::sensHall, -1, 30);

    // Set up electrical measurements readout
    hbpOut.scheduleRead(HoverboardAPI::Codes::sensElectrical, -1, 100);

    // Send PWM values periodically
    hbpOut.updateParamVariable(HoverboardAPI::Codes::setPointPWM, &PWMData, sizeof(PWMData));
    hbpOut.scheduleTransmission(HoverboardAPI::Codes::setPointPWM, -1, 30);
  #endif



  #if defined(OUTPUT_ESPNOW) || defined(INPUT_ESPNOW)
    esp_now_register_recv_cb(espReceiveDataWrapper);
  #endif

  #if defined(OUTPUT_ESPNOW) || defined(OUTPUT_PROTOCOL_UART)
    hbpOut.updateParamHandler(HoverboardAPI::Codes::sensHall, processHalldata);
    hbpOut.scheduleTransmission(HoverboardAPI::Codes::setPointPWM, -1, 30);
    hbpOut.scheduleRead(HoverboardAPI::Codes::protocolCountSum, -1, 30);
  #endif

}

void loopCommunication( void *pvparameters ) {
//  int taskno = (int)pvparameters;
  while(1) {

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
    COM[DEBUG_COM]->print(hbpOut.getBatteryVoltage());
    COM[DEBUG_COM]->print(" Current0: ");
    COM[DEBUG_COM]->print(hbpOut.getMotorAmpsAvg(0));
    COM[DEBUG_COM]->print(" Current1: ");
    COM[DEBUG_COM]->print(hbpOut.getMotorAmpsAvg(1));
    COM[DEBUG_COM]->print(" Speed: ");
    COM[DEBUG_COM]->print(hbpOut.getSpeed_kmh());
    COM[DEBUG_COM]->print(" Steer: ");
    COM[DEBUG_COM]->print(hbpOut.getSteer_kmh());
    COM[DEBUG_COM]->println();
  #endif


    PWMData.pwm[0] = motor.setpoint.pwm + motor.setpoint.steer;
    PWMData.pwm[1] = motor.setpoint.pwm - motor.setpoint.steer;

    // Send Buzzer Data
    // TODO: Find better way to find out when to send data. This way edge case 0, 0, 0 can not be sent.
    if( (sendBuzzer.buzzerFreq != 0) || (sendBuzzer.buzzerLen != 0) || (sendBuzzer.buzzerPattern != 0) ) {
      hbpOut.sendBuzzer(sendBuzzer.buzzerFreq, sendBuzzer.buzzerPattern, sendBuzzer.buzzerLen);

      sendBuzzer.buzzerFreq = 0;
      sendBuzzer.buzzerLen = 0;
      sendBuzzer.buzzerPattern = 0;
    }

#endif

#ifdef OUTPUT_PROTOCOL_UART
    pollUART();
    hbpOut.protocolTick();
#endif

#ifdef OUTPUT_ESPNOW
    hbpOut.protocolTick();
#endif

#ifdef INPUT_ESPNOW
    hbpIn.protocolTick();
#endif

    unsigned long start = millis();
    while (millis() < start + MOTORINPUT_PERIOD){
    #ifdef OUTPUT_PROTOCOL_UART
        pollUART();
        hbpOut.protocolTick();
    #endif

    #ifdef OUTPUT_ESPNOW
        hbpOut.protocolTick();
    #endif

    #ifdef INPUT_ESPNOW
        hbpIn.protocolTick();
    #endif

      delayMicroseconds(100);
    }
  }
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
