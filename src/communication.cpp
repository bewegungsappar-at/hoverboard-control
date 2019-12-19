#include "communication.h"
#include <Arduino.h>
#include "main.h"
#include "config.h"
#include "serialbridge.h"
#include <HoverboardAPI.h>
#include "protocolFunctions.h"

#include "ESP32_espnow_MasterSlave.h"
#include "input.h"
#include <esp_now.h>

#ifdef ODROID_GO_HW
#include "go_display.h"
#endif // ODROID_GO_HW


#include <WiFi.h>
#include <WiFiUdp.h>

unsigned int localPort = 1337; // local port to listen for UDP packets

IPAddress broadcast(192,168,0,255); //UDP Broadcast IP data sent to all devicess on same network


// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;


//======================================================================
///////////////////////////////////////////////////////////
// Global Variables
///////////////////////////////////////////////////////////

volatile PROTOCOL_BUZZER_DATA sendBuzzer = {
    .buzzerFreq = 0,
    .buzzerPattern = 0,
    .buzzerLen = 0,
};

PROTOCOL_PWM_DATA PWMData = {
#ifdef INPUT_TESTRUN
    .pwm = {0,0},
    .speed_max_power =  1000,
    .speed_min_power = -1000,
    .speed_minimum_pwm = 1  // guard value, below this set to zero
#else
    .pwm = {0,0},
    .speed_max_power =  400,
    .speed_min_power = -400,
    .speed_minimum_pwm = 30 // guard value, below this set to zero
#endif
};

PROTOCOL_SPEED_DATA PIDSpeedData = {
  .wanted_speed_mm_per_sec = {0,0},
  .speed_max_power = 400, // max speed in this mode
  .speed_min_power = 400, // minimum speed (to get wheels moving)
  .speed_minimum_speed = 30 // below this, we don't ask it to do anything
};

uint8_t enableHoverboardMotors = 0;

int scanCounter = 0;

HoverboardAPI hbpOut;
HoverboardAPI hbpIn;

///////////////////////////////////////////////////////////
// Support Functions
///////////////////////////////////////////////////////////

void protocolMarkup(unsigned char *data, int len, int prefix) {
  bool charOut = false;
  switch (prefix) {
    case 0:
      COM[DEBUG_COM]->print("Out UART   ");
      break;
    case 1:
      COM[DEBUG_COM]->print("Out ESPnow ");
      break;
    case 2:
      COM[DEBUG_COM]->print("In  ESPnow ");
      break;
    case 3:
      COM[DEBUG_COM]->print("Out UDP    ");
      break;
    default:
      COM[DEBUG_COM]->printf("if:%01i ", prefix);
      break;
  }

  for(int i = 0; i< len; i++) {
    switch (i) {
    case 0:
      COM[DEBUG_COM]->printf("SOM:%01i ", data[i]);
      break;

    case 1:
      COM[DEBUG_COM]->printf("CMD:%c ", (data[i] & 0b01111111) );
      COM[DEBUG_COM]->printf("ACK:%01i ", (data[i] & 0b10000000) );
      break;

    case 2:
      COM[DEBUG_COM]->printf("CI:%03i ", data[i]);
      break;

    case 3:
      COM[DEBUG_COM]->printf("len:%03i ", data[i]);
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
      } else if(data[i] == HoverboardAPI::Codes::protocolVersion) {
        COM[DEBUG_COM]->print("Version   ");
      } else if(data[i] == HoverboardAPI::Codes::text) {
        COM[DEBUG_COM]->print("Text      ");
        charOut = true;
      } else {
        COM[DEBUG_COM]->printf("Code:0x%02X ", data[i]);
      }
      break;

    default:
      if(i==len-1) {
        COM[DEBUG_COM]->printf("CS:0x%02X ", data[i]);
        charOut = false;
      } else if (charOut) {
        COM[DEBUG_COM]->printf("%c", data[i]);
      } else {
        COM[DEBUG_COM]->printf("%02X ", data[i]);
      }
      break;
    }
  }
  COM[DEBUG_COM]->println();
}

int serialWriteWrapper(unsigned char *data, int len)
{
  #ifdef DEBUG_PROTOCOL_OUTGOING_MARKUP
  protocolMarkup(data, len, 0);
  #endif

  return (int) COM[MOTOR_COM]->write(data,len);
}

int udpSendDataWrapper(unsigned char *data, int len) {

  #ifdef DEBUG_PROTOCOL_OUTGOING_MARKUP
  protocolMarkup(data, len, 3);
  #endif

  udp.beginPacket(broadcast, localPort);
  size_t result = udp.write((const uint8_t *) data,(size_t) len); //Send one byte to ESP8266
  udp.endPacket();

  return (int) result;
}


int espSendDataWrapper(unsigned char *data, int len) {

  #ifdef DEBUG_PROTOCOL_OUTGOING_MARKUP
  protocolMarkup(data, len, 1);
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
    extern bool debug_espnow;
    if(debug_espnow) COM[DEBUG_COM]->print("\t\tLast Packet Recv from: "); if(debug_espnow) COM[DEBUG_COM]->println(macStr);
    if(debug_espnow) COM[DEBUG_COM]->print("\t\tLast Packet Recv Data: "); if(debug_espnow) COM[DEBUG_COM]->write(data, data_len);
    if(debug_espnow) COM[DEBUG_COM]->println("");
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

  #ifdef DEBUG_PROTOCOL_OUTGOING_MARKUP
    protocolMarkup((unsigned char *)data, data_len, 2);
  #endif

  // Pass data to protocol
  for(int i=0; i < data_len; i++)
  {
    if( communicationSettings.output == COMM_OUT_ESPNOW) hbpOut.protocolPush(data[i]);
    if( communicationSettings.input  == COMM_IN_ESPNOW ) hbpIn.protocolPush(data[i]);
  }
}

#ifdef WIFI
  #include <WiFi.h>
  extern WiFiClient TCPClient[NUM_COM][MAX_NMEA_CLIENTS];
#endif


double limit(double min, double value, double max) {
  if(value<min) value = min;
  if(value>max) value = max;
  return value;
}

void processHalldata ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg )
{
  fn_defaultProcessing(s, param, cmd, msg);

  switch (cmd) {
    case PROTOCOL_CMD_READVALRESPONSE:
    case PROTOCOL_CMD_WRITEVAL:
      motor.measured.actualSpeed_kmh = hbpOut.getSpeed_kmh();
      motor.measured.actualSteer_kmh = hbpOut.getSteer_kmh();
      break;
  }
}

void relayDataOut ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg );
void relayDataIn ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg );

void processPWMdata ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
  switch (cmd) {
    case PROTOCOL_CMD_WRITEVAL:
    case PROTOCOL_CMD_READVALRESPONSE:
      if( communicationSettings.input == COMM_IN_ESPNOW || communicationSettings.input == COMM_IN_UDP)
      {
        // Relay messages coming from hbpIn (ESP Now, remote) to hbpOut (Hoverboard, UART)
        if(hbpIn.s.params[HoverboardAPI::Codes::setPointPWM]) hbpIn.updateParamHandler( HoverboardAPI::Codes::setPointPWM ,relayDataOut);
      }
      break;

    case PROTOCOL_CMD_READVAL:
    case PROTOCOL_CMD_SILENTREAD:
      PWMData.pwm[0] = motor.setpoint.pwm + motor.setpoint.steer;
      PWMData.pwm[1] = motor.setpoint.pwm - motor.setpoint.steer;
      break;
  }
  fn_defaultProcessing(s, param, cmd, msg);
}

void processPIDSpeedData ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
  switch (cmd) {
    case PROTOCOL_CMD_WRITEVAL:
    case PROTOCOL_CMD_READVALRESPONSE:
      if( communicationSettings.input == COMM_IN_ESPNOW || communicationSettings.input == COMM_IN_UDP)
      {
        // Relay messages coming from hbpIn (ESP Now, remote) to hbpOut (Hoverboard, UART)
        if(hbpIn.s.params[HoverboardAPI::Codes::setSpeed]) hbpIn.updateParamHandler( HoverboardAPI::Codes::setSpeed ,relayDataOut);
      }
      break;

    case PROTOCOL_CMD_READVAL:
    case PROTOCOL_CMD_SILENTREAD:
      PIDSpeedData.wanted_speed_mm_per_sec[0] = motor.setpoint.pwm + motor.setpoint.steer;
      PIDSpeedData.wanted_speed_mm_per_sec[1] = motor.setpoint.pwm - motor.setpoint.steer;
      break;
  }
  fn_defaultProcessing(s, param, cmd, msg);
}

void pollUART() {
  // Read and Process Incoming data
  int i=0;
  while(COM[MOTOR_COM]->available() && i++ < 1024) { // read maximum 1024 byte at once.
    unsigned char readChar = COM[MOTOR_COM]->read();
    hbpOut.protocolPush( readChar );
  }
    }

void pollUDP()
{
  if (udp.parsePacket())
  {
    while( udp.available() )
    {
      unsigned char readChar = (unsigned char) udp.read();

      if( communicationSettings.input == COMM_IN_UDP) hbpIn.protocolPush( readChar );
      if( communicationSettings.output == COMM_OUT_UDP) hbpOut.protocolPush( readChar );
    }
  }
}

void relayDataOut ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg )
{
  hbpOut.protocolPost(msg);
}

void relayDataIn ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg )
{
  hbpIn.protocolPost(msg);
}

void consoleLog ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg )
{
  switch (cmd) {
    case PROTOCOL_CMD_WRITEVAL:
    case PROTOCOL_CMD_READVALRESPONSE:

      if( (param) && (strlen((const char *)msg->content) <= param->len ) )
        COM[DEBUG_COM]->println( (char *) msg->content);
      break;
  }
}


///////////////////////////////////////////////////////////
// Communication Init
///////////////////////////////////////////////////////////

void initializeUDP()
{
  if( communicationSettings.input == COMM_IN_UDP)
  {
    WiFi.softAP(communicationSettings.wifi_ssid, communicationSettings.wifi_pass, 13, 1);    //Create Access point on Channel 13 with hidden ssid

    if(debug) COM[DEBUG_COM]->print("IP address: ");
    if(debug) COM[DEBUG_COM]->println(WiFi.softAPIP());

    broadcast = WiFi.softAPIP();

    broadcast[3] = 255;
    if(debug) COM[DEBUG_COM]->print("Broadcast address: ");
    if(debug) COM[DEBUG_COM]->println(broadcast);
  }
  else if( communicationSettings.output == COMM_OUT_UDP)
  {
    WiFi.begin(communicationSettings.wifi_ssid, communicationSettings.wifi_pass);   //Connect to access point

    // Wait for connection
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      if(debug) COM[DEBUG_COM]->print(".");
    }

    if(debug) COM[DEBUG_COM]->println("");
    if(debug) COM[DEBUG_COM]->print("Connected to ");
    if(debug) COM[DEBUG_COM]->println(communicationSettings.wifi_ssid);
    if(debug) COM[DEBUG_COM]->print("IP address: ");
    if(debug) COM[DEBUG_COM]->println(WiFi.localIP());

    broadcast = WiFi.localIP();

    broadcast[3] = 255;
    if(debug) COM[DEBUG_COM]->print("Broadcast address: ");
    if(debug) COM[DEBUG_COM]->println(broadcast);
  }

  //Start UDP
  if(debug) COM[DEBUG_COM]->println("Starting UDP");
  udp.begin(localPort);
  if(debug) COM[DEBUG_COM]->print("Local port: ");
  if(debug) COM[DEBUG_COM]->println(localPort);

  broadcast = WiFi.softAPIP();

  broadcast[3] = 255;
  if(debug) COM[DEBUG_COM]->print("Broadcast address: ");
  if(debug) COM[DEBUG_COM]->println(broadcast);
  delay(100);
}

void initializeOdroidGo()
{
  #ifdef ODROID_GO_HW
    // init display and show labels
    GO_DISPLAY::setup();
    GO_DISPLAY::connectionSelector();
  #endif // ODROID_GO_HW
}

void initializeESPnow()
{
  // Init ESPnow
  setupEspNow();
  esp_now_register_recv_cb(espReceiveDataWrapper);
  hbpOut.sendPing(); // First messages are lost
  hbpOut.sendPing();
  hbpOut.sendPing();
}

void setupRelaying()
{
  // ESPnow to UART Protocol Relay
  // Relay messages coming from hbpOut (Hoverboard, UART) to hbpIn (ESP Now, remote)
  for (int i = 0; i < sizeof(hbpOut.s.params)/sizeof(hbpOut.s.params[0]); i++) {
    if(hbpOut.s.params[i]) hbpOut.updateParamHandler((HoverboardAPI::Codes) i ,relayDataIn);
  }

  // Relay messages coming from hbpIn (ESP Now, remote) to hbpOut (Hoverboard, UART)
  for (int i = 0; i < sizeof(hbpIn.s.params)/sizeof(hbpIn.s.params[0]); i++) {
    if(hbpIn.s.params[i]) hbpIn.updateParamHandler((HoverboardAPI::Codes) i ,relayDataOut);
  }
}

void setupPWMtransmission();
void schedulePeriodicReadings();


void setupHbpOut()
{
  // Initialize and setup protocol values, setup all periodic messages.

    // Print all incoming Texts on console
    hbpOut.updateParamHandler(HoverboardAPI::Codes::text, consoleLog);


  // Set up hall data readout (=hoverboard measured speed)
  hbpOut.updateParamHandler(HoverboardAPI::Codes::sensHall, processHalldata);

#ifdef DEBUG_SPEED
  hbpOut.updateParamVariable( HoverboardAPI::Codes::setSpeed, &PIDSpeedData, sizeof(PIDSpeedData.wanted_speed_mm_per_sec)); // Perform short write
  hbpOut.updateParamHandler(  HoverboardAPI::Codes::setSpeed, processPIDSpeedData);
#else
  hbpOut.updateParamVariable( HoverboardAPI::Codes::setPointPWM, &PWMData, sizeof(PWMData.pwm));
  hbpOut.updateParamVariable( HoverboardAPI::Codes::setPointPWMData, &PWMData, sizeof(PWMData));
  hbpOut.updateParamHandler(  HoverboardAPI::Codes::setPointPWM, processPWMdata);
#endif

#ifndef DEBUG_DISABLE_PWMOUTPUT
  setupPWMtransmission();
  schedulePeriodicReadings();
#endif
}

void schedulePeriodicReadings()
{
  // Get Protocol statistics
    hbpOut.scheduleRead(HoverboardAPI::Codes::protocolCountSum, -1, 1000);

  // hall data (=hoverboard measured speed)
    hbpOut.scheduleRead(HoverboardAPI::Codes::sensHall, -1, 100);

    // Set up electrical measurements readout
    hbpOut.scheduleRead(HoverboardAPI::Codes::sensElectrical, -1, 500);
}

void setupPWMtransmission()
{
  // Initialize  PWM limits
#ifdef DEBUG_SPEED
  hbpOut.sendSpeedData(PIDSpeedData.wanted_speed_mm_per_sec[0], PIDSpeedData.wanted_speed_mm_per_sec[1], PIDSpeedData.speed_max_power, PIDSpeedData.speed_minimum_speed, PROTOCOL_SOM_ACK);
#else
  hbpOut.sendPWMData(PWMData.pwm[0], PWMData.pwm[1], PWMData.speed_max_power, PWMData.speed_min_power, PWMData.speed_minimum_pwm, PROTOCOL_SOM_ACK);
#endif

#ifdef DEBUG_SPEED
  hbpOut.scheduleTransmission(HoverboardAPI::Codes::setSpeed, -1, 30);
  hbpOut.sendPIDControl(22,1,8,100,PROTOCOL_SOM_ACK);
#else
  hbpOut.scheduleTransmission(HoverboardAPI::Codes::setPointPWM, -1, 30);
#endif

#ifdef INPUT_TESTRUN
    // send enable periodically
    hbpOut.updateParamVariable(HoverboardAPI::Codes::enableMotors, &enableHoverboardMotors, sizeof(enableHoverboardMotors));
    hbpOut.scheduleTransmission(HoverboardAPI::Codes::enableMotors, -1, 60);
#else
  // enable motors
  hbpOut.sendEnable(1, PROTOCOL_SOM_ACK);
#endif
}

void processOdroidGo()
{
#ifdef ODROID_GO_HW
  GO.update();

  static int state = 0;

  static int BtnMenuOld = 0;
  bool BtnMenuJustPressed = ( GO.BtnMenu.isPressed() && BtnMenuOld == 0 ) ;
  BtnMenuOld = GO.BtnMenu.isPressed();


  switch (state)
  {
    case 0:   // Init monitor screen
        GO.lcd.setTextSize(1);
        GO.lcd.setFreeFont(&FreeMono9pt7b);
        GO.lcd.clearDisplay();
        GO_DISPLAY::show_labels();
        state = 1;
    case 1:   // Show monitor screen
    {

  #ifdef DEBUG_PING
    static int pingCounter = 0;
    if( pingCounter++ >= (1000 / MOTORINPUT_PERIOD) ) {
      pingCounter = 0;
      GO_DISPLAY::show_internal_battery_voltage();
      GO_DISPLAY::plot(latency);
      latency = 0;
      hbpOut.sendPing();
    }
  #endif
    static int16_t tempPID = 100;

    // TODO: assuming motor 0 is left and motor 1 is right
    GO_DISPLAY::set(GO_DISPLAY::CURRENT_LEFT ,hbpOut.getMotorAmpsAvg(0));
    GO_DISPLAY::set(GO_DISPLAY::CURRENT_RIGHT ,hbpOut.getMotorAmpsAvg(1));
    GO_DISPLAY::set(GO_DISPLAY::SPEED, hbpOut.getSpeed_kmh());
    GO_DISPLAY::set(GO_DISPLAY::STEER, hbpOut.getSteer_kmh());
    GO_DISPLAY::set(GO_DISPLAY::PWM_LEFT, PWMData.pwm[0]);
    GO_DISPLAY::set(GO_DISPLAY::PWM_RIGHT, PWMData.pwm[1]);
    GO_DISPLAY::set(GO_DISPLAY::BATTERY_VOLTAGE, hbpOut.getBatteryVoltage());
    GO_DISPLAY::set(GO_DISPLAY::PACKAGE_LOSS_DOWNSTREAM, (float) latency);
    GO_DISPLAY::set(GO_DISPLAY::PACKAGE_LOSS_UPSTREAM, (float) tempPID);

    GO_DISPLAY::plotBattery(hbpOut.getBatteryVoltage());
    GO_DISPLAY::plotSpeed(hbpOut.getSpeed_kmh());

    // TODO: just to see if something happens

    double wantedSpeed = 0.0;
    double wantedSteer = 0.0;

    if(GO.JOY_Y.isAxisPressed() == 2) wantedSpeed =  200.0;
    if(GO.JOY_Y.isAxisPressed() == 1) wantedSpeed = -200.0;
    if(GO.JOY_X.isAxisPressed() == 1) wantedSteer =  200.0;
    if(GO.JOY_X.isAxisPressed() == 2) wantedSteer = -200.0;

    if(GO.BtnA.isPressed()) wantedSpeed = wantedSpeed *2.0;
    if(GO.BtnB.isPressed()) wantedSpeed = wantedSpeed *2.0;

    if(wantedSpeed > 400) wantedSpeed = 400;
    if(wantedSpeed < -400) wantedSpeed = -400;

    slowReset(motor.setpoint.pwm,   wantedSpeed, 0, 0.15);
    slowReset(motor.setpoint.steer, wantedSteer, 0, 0.5);

    if(GO.BtnStart.isPressed()) hbpOut.sendPing();

#ifdef DEBUG_SPEED
      if(BtnMenuJustPressed) hbpOut.sendPIDControl(22,1,8,--tempPID,PROTOCOL_SOM_ACK);
      if(GO.BtnVolume.isPressed()) hbpOut.sendPIDControl(22,1,8,++tempPID,PROTOCOL_SOM_ACK);
# else
      if(BtnMenuJustPressed) state = 2;
#endif

      break;
    }

    case 2: // Init Menu
      GO.lcd.setTextSize(1);
      GO.lcd.setFreeFont(&FreeMono9pt7b);
      GO.lcd.clearDisplay();
      state = 3;

    case 3: //Process Menu
      if(BtnMenuJustPressed) state = 0;  // Go back
      break;


  default:
    state = 0;
    break;
  }




  #endif // ODROID_GO_HW
}

void printProtocolMeasurements()
{
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
}

void processBuzzer()
{
  // Send Buzzer Data
  // TODO: Find better way to find out when to send data. This way edge case 0, 0, 0 can not be sent.
  if( (sendBuzzer.buzzerFreq != 0) || (sendBuzzer.buzzerLen != 0) || (sendBuzzer.buzzerPattern != 0) ) {
    hbpOut.sendBuzzer(sendBuzzer.buzzerFreq, sendBuzzer.buzzerPattern, sendBuzzer.buzzerLen);

    sendBuzzer.buzzerFreq = 0;
    sendBuzzer.buzzerLen = 0;
    sendBuzzer.buzzerPattern = 0;
  }
}

void receiveAndprocessProtocol()
{
  if( communicationSettings.output == COMM_OUT_UART )
    pollUART();

  if( communicationSettings.output == COMM_OUT_UDP || communicationSettings.input == COMM_IN_UDP )
    pollUDP();

  hbpOut.protocolTick();

  if( communicationSettings.input == COMM_IN_ESPNOW || communicationSettings.input == COMM_IN_UDP )
    hbpIn.protocolTick();
}



void setupCommunication()
{

  initializeOdroidGo();

  if( communicationSettings.output == COMM_OUT_ESPNOW )
  {
    initializeESPnow();
    hbpOut.setSendSerialData(espSendDataWrapper);
  }
  else if( communicationSettings.output == COMM_OUT_UDP )
  {
    initializeUDP();
    hbpOut.setSendSerialData(udpSendDataWrapper);
  }
  else
  {
    hbpOut.setSendSerialData(serialWriteWrapper);
  }

  if( communicationSettings.input == COMM_IN_ESPNOW )
  {
    initializeESPnow();
    setupRelaying();
    hbpIn.setSendSerialData(espSendDataWrapper);
  }
  else if( communicationSettings.input == COMM_IN_UDP )
  {
    initializeUDP();
    setupRelaying();
    hbpIn.setSendSerialData(udpSendDataWrapper);
  }
  else
  {
    setupHbpOut();
  }
}

void loopCommunication( void *pvparameters )
{
  while(1)
  {
    processOdroidGo();
    printProtocolMeasurements();

    unsigned long start = millis();
    do
    {
      receiveAndprocessProtocol();
      delayMicroseconds(100);
    }
    while( millis() < start + MOTORINPUT_PERIOD );
  }
}
