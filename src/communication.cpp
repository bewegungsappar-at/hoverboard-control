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

///////////////////////////////////////////////////////////
// Global Variables
///////////////////////////////////////////////////////////

unsigned int localPort = 1337; // local port to listen for UDP packets
IPAddress broadcast(192,168,0,255); //UDP Broadcast IP data sent to all devicess on same network
WiFiUDP udp; // A UDP instance to let us send and receive packets over UDP


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
    .speed_max_power =  500,
    .speed_min_power = -500,
    .speed_minimum_pwm = 20 // guard value, below this set to zero
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

PROTOCOL_ADC_SETTINGS protocolADCSettings;
int adcSettingsDelta = 0;

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
    if( sysconfig.chan_out == COMM_CHAN_ESPNOW ) hbpOut.protocolPush(data[i]);
    if( sysconfig.chan_in  == COMM_CHAN_ESPNOW ) hbpIn.protocolPush(data[i]);
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
    case PROTOCOL_CMD_READVAL:
    case PROTOCOL_CMD_SILENTREAD:
      slowReset(PWMData.pwm[0], motor.setpoint.pwm + motor.setpoint.steer, 50, 0.0);
      slowReset(PWMData.pwm[1], motor.setpoint.pwm - motor.setpoint.steer, 50, 0.0);
      break;
  }
  fn_defaultProcessing(s, param, cmd, msg);
}

void waitForMessage ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
  switch (cmd) {
    case PROTOCOL_CMD_WRITEVAL:
    case PROTOCOL_CMD_READVALRESPONSE:
      if( sysconfig.chan_in == COMM_CHAN_ESPNOW || sysconfig.chan_in == COMM_CHAN_UDP)
      {
        // Relay messages coming from hbpIn (ESP Now, remote) to hbpOut (Hoverboard, UART)
        if(hbpIn.s.params[HoverboardAPI::Codes::setPointPWM]) hbpIn.updateParamHandler( HoverboardAPI::Codes::setPointPWM ,relayDataOut);
        hbpOut.scheduleTransmission(HoverboardAPI::Codes::setPointPWM, 0, 30);
        if(hbpIn.s.params[HoverboardAPI::Codes::setSpeed]) hbpIn.updateParamHandler( HoverboardAPI::Codes::setSpeed ,relayDataOut);
        hbpOut.scheduleTransmission(HoverboardAPI::Codes::setSpeed, 0, 30);
      }
      break;
  }
}

void processPIDSpeedData ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
  switch (cmd) {
    case PROTOCOL_CMD_READVAL:
    case PROTOCOL_CMD_SILENTREAD:
      slowReset(PIDSpeedData.wanted_speed_mm_per_sec[0], motor.setpoint.pwm + motor.setpoint.steer, 50, 0.0);
      slowReset(PIDSpeedData.wanted_speed_mm_per_sec[1], motor.setpoint.pwm - motor.setpoint.steer, 50, 0.0);
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

      if( sysconfig.chan_in == COMM_CHAN_UDP) hbpIn.protocolPush( readChar );
      if( sysconfig.chan_out == COMM_CHAN_UDP) hbpOut.protocolPush( readChar );
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
  if( sysconfig.chan_in == COMM_CHAN_UDP)
  {
    WiFi.softAP(sysconfig.wifi_ssid, sysconfig.wifi_pass, 9 , 1);    //Create Access point on Channel 13 with hidden ssid

    if(debug) COM[DEBUG_COM]->print("IP address: ");
    if(debug) COM[DEBUG_COM]->println(WiFi.softAPIP());

    broadcast = WiFi.softAPIP();

    broadcast[3] = 255;
    if(debug) COM[DEBUG_COM]->print("Broadcast address: ");
    if(debug) COM[DEBUG_COM]->println(broadcast);
  }
  else if( sysconfig.chan_out == COMM_CHAN_UDP)
  {
    WiFi.begin(sysconfig.wifi_ssid, sysconfig.wifi_pass);   //Connect to access point

    // Wait for connection
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      if(debug) COM[DEBUG_COM]->print(".");
    }

    if(debug) COM[DEBUG_COM]->println("");
    if(debug) COM[DEBUG_COM]->print("Connected to ");
    if(debug) COM[DEBUG_COM]->println(sysconfig.wifi_ssid);
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

void hbpoutSetupHandlers()
{
  // Print all incoming Texts on console
  hbpOut.updateParamHandler(HoverboardAPI::Codes::text, consoleLog);

  // Set up hall data readout (=hoverboard measured speed)
  hbpOut.updateParamHandler(HoverboardAPI::Codes::sensHall, processHalldata);
}


void processADCsettings ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
  fn_defaultProcessing(s, param, cmd, msg);
  switch (cmd) {
    case PROTOCOL_CMD_READVALRESPONSE:
    case PROTOCOL_CMD_WRITEVAL:
      protocolADCSettings.adc_off_end += adcSettingsDelta;
      protocolADCSettings.adc1_zero   += adcSettingsDelta;
      hbpOut.sendRawData( PROTOCOL_CMD_WRITEVAL, (unsigned char)HoverboardAPI::Codes::adcSettings, (unsigned char*) &protocolADCSettings, sizeof(protocolADCSettings), PROTOCOL_SOM_ACK );
      break;
  }
}

void hbpoutSetupADCsettings()
{
  hbpOut.updateParamVariable( HoverboardAPI::Codes::adcSettings, &protocolADCSettings, sizeof(protocolADCSettings) );
  hbpOut.updateParamHandler( HoverboardAPI::Codes::adcSettings, processADCsettings );
}


void hbpoutScheduleReadings()
{
  // Get Protocol statistics
  hbpOut.scheduleRead(HoverboardAPI::Codes::protocolCountSum, -1, 1000);

  // hall data (=hoverboard measured speed)
  hbpOut.scheduleRead(HoverboardAPI::Codes::sensHall, -1, 100);

  // Set up electrical measurements readout
  hbpOut.scheduleRead(HoverboardAPI::Codes::sensElectrical, -1, 500);
}

void hbpoutSetupPWMtransmission()
{
  hbpOut.updateParamVariable( HoverboardAPI::Codes::setSpeed, &PIDSpeedData, sizeof(PIDSpeedData.wanted_speed_mm_per_sec)); // Perform short write
  hbpOut.updateParamHandler(  HoverboardAPI::Codes::setSpeed, processPIDSpeedData);

  hbpOut.updateParamVariable( HoverboardAPI::Codes::setPointPWM, &PWMData, sizeof(PWMData.pwm));
  hbpOut.updateParamVariable( HoverboardAPI::Codes::setPointPWMData, &PWMData, sizeof(PWMData));
  hbpOut.updateParamHandler(  HoverboardAPI::Codes::setPointPWM, processPWMdata);

#ifdef DEBUG_SPEED
  hbpOut.sendSpeedData(PIDSpeedData.wanted_speed_mm_per_sec[0], PIDSpeedData.wanted_speed_mm_per_sec[1], PIDSpeedData.speed_max_power, PIDSpeedData.speed_minimum_speed, PROTOCOL_SOM_ACK);
  hbpOut.scheduleTransmission(HoverboardAPI::Codes::setSpeed, -1, 30);
  hbpOut.sendPIDControl(22,1,8,100,PROTOCOL_SOM_ACK);
#else
  hbpOut.sendPWMData(PWMData.pwm[0], PWMData.pwm[1], PWMData.speed_max_power, PWMData.speed_min_power, PWMData.speed_minimum_pwm, PROTOCOL_SOM_ACK);
  hbpOut.scheduleTransmission(HoverboardAPI::Codes::setPointPWM, -1, 30);
#endif

#ifdef INPUT_TESTRUN
    // send enable periodically
    hbpOut.updateParamVariable(HoverboardAPI::Codes::enableMotors, &enableHoverboardMotors, sizeof(enableHoverboardMotors));
    hbpOut.scheduleTransmission(HoverboardAPI::Codes::enableMotors, -1, 60);
#else
  // enable motors
  hbpOut.sendEnable(1, PROTOCOL_SOM_ACK);
  hbpOut.sendEnable(1, PROTOCOL_SOM_ACK); // TODO: Check why this workaround is neccessary.
#endif
  hbpIn.updateParamHandler(  HoverboardAPI::Codes::setSpeed, waitForMessage);
  hbpIn.updateParamHandler(  HoverboardAPI::Codes::setPointPWM, waitForMessage);
}

void processOdroidGo()
{
#ifdef ODROID_GO_HW
  GO.update();

  typedef enum
  {
      OD_LCD_MONITORINIT,
      OD_LCD_MONITOR,
      OD_LCD_MENUINIT,
      OD_LCD_MENU
  } odroidLCD_state;

  static odroidLCD_state state = OD_LCD_MONITORINIT;

  static int BtnMenuOld = 0;
  bool BtnMenuJustPressed = ( GO.BtnMenu.isPressed() && BtnMenuOld == 0 ) ;
  BtnMenuOld = GO.BtnMenu.isPressed();


  switch (state)
  {
    case OD_LCD_MONITORINIT:
        GO.lcd.setTextSize(1);
        GO.lcd.setFreeFont(&FreeMono9pt7b);
        GO.lcd.clearDisplay();
        GO_DISPLAY::show_labels();
        state = OD_LCD_MONITOR;

    case OD_LCD_MONITOR:
    {
# ifdef DEBUG_PING
      static int pingCounter = 0;
      if( pingCounter++ >= (1000 / MOTORINPUT_PERIOD) ) {
        pingCounter = 0;
        GO_DISPLAY::show_internal_battery_voltage();
        GO_DISPLAY::plot(latency);
        latency = 0;
        hbpOut.sendPing();
      }
# endif

      static int16_t tempPID = 100;

      double deltaRtoSpeed = paddelec.cfgPaddle.deltaRtoSpeed * 10000.0;
      double pwmMultiplier = paddelec.cfgPaddle.pwmMultiplier * 100.0;

      // TODO: assuming motor 0 is left and motor 1 is right
      GO_DISPLAY::set(GO_DISPLAY::CURRENT_LEFT ,hbpOut.getMotorAmpsAvg(0));
      GO_DISPLAY::set(GO_DISPLAY::CURRENT_RIGHT ,hbpOut.getMotorAmpsAvg(1));
      GO_DISPLAY::set(GO_DISPLAY::SPEED, hbpOut.getSpeed_kmh());
      GO_DISPLAY::set(GO_DISPLAY::STEER, hbpOut.getSteer_kmh());
      GO_DISPLAY::set(GO_DISPLAY::PWM_LEFT, PWMData.pwm[0]);
      GO_DISPLAY::set(GO_DISPLAY::PWM_RIGHT, deltaRtoSpeed);
      GO_DISPLAY::set(GO_DISPLAY::BATTERY_VOLTAGE, hbpOut.getBatteryVoltage());
      GO_DISPLAY::set(GO_DISPLAY::PACKAGE_LOSS_DOWNSTREAM, (float) latency);
      GO_DISPLAY::set(GO_DISPLAY::PACKAGE_LOSS_UPSTREAM, pwmMultiplier);

      GO_DISPLAY::plotBattery(hbpOut.getBatteryVoltage());
      GO_DISPLAY::plotSpeed(hbpOut.getSpeed_kmh());

      double odroidSpeed = 0.0;
      double odroidSteer = 0.0;

      if(GO.JOY_Y.isAxisPressed() == 2) odroidSpeed =  PWMData.speed_max_power / 3;
      if(GO.JOY_Y.isAxisPressed() == 1) odroidSpeed = -PWMData.speed_max_power / 3;
      if(GO.JOY_X.isAxisPressed() == 1) odroidSteer =  PWMData.speed_max_power / 3;
      if(GO.JOY_X.isAxisPressed() == 2) odroidSteer = -PWMData.speed_max_power / 3;

      if(GO.BtnA.isPressed()) odroidSpeed = odroidSpeed *2.0;
      if(GO.BtnB.isPressed()) odroidSpeed = odroidSpeed *2.0;

      if(odroidSpeed > PWMData.speed_max_power) odroidSpeed =   PWMData.speed_max_power;
      if(odroidSpeed < -PWMData.speed_max_power) odroidSpeed = -PWMData.speed_max_power;


      if(nunchukState != RUNNING)
      {
        slowReset(motor.setpoint.pwm,   odroidSpeed, 0, 0.15);
        slowReset(motor.setpoint.steer, odroidSteer, 0, 0.2);
      }

      if(GO.BtnStart.isPressed()) hbpOut.sendPing();

# ifdef DEBUG_SPEED
        if(BtnMenuJustPressed) hbpOut.sendPIDControl(22,1,8,--tempPID,PROTOCOL_SOM_ACK);
        if(GO.BtnVolume.isPressed()) hbpOut.sendPIDControl(22,1,8,++tempPID,PROTOCOL_SOM_ACK);
# else
        if(BtnMenuJustPressed) state = OD_LCD_MENUINIT;
# endif

      break;
    }

    case OD_LCD_MENUINIT:
      GO_DISPLAY::menu(true);
      state = OD_LCD_MENU;

    case OD_LCD_MENU:
      switch (GO_DISPLAY::menu(false))
      {
      case GO_DISPLAY::MENU_SENSOR:
        hbpOut.sendEnable(1, PROTOCOL_SOM_ACK); //TODO: workaround..
        hbpoutScheduleReadings();
        state = OD_LCD_MONITORINIT;
        break;

      case GO_DISPLAY::MENU_PWM:
        hbpoutSetupPWMtransmission();
        state = OD_LCD_MONITORINIT;
        break;

      case GO_DISPLAY::MENU_ENABLE:
        hbpOut.sendEnable(1, PROTOCOL_SOM_ACK);
        hbpOut.sendEnable(1, PROTOCOL_SOM_ACK); //TODO: workaround..
        state = OD_LCD_MONITORINIT;
        break;

      case GO_DISPLAY::MENU_DISABLE:
        hbpOut.sendEnable(0, PROTOCOL_SOM_ACK);
        state = OD_LCD_MONITORINIT;
        break;

      case GO_DISPLAY::MENU_WIRESHARKDEC:
        hbpoutSetupADCsettings();
        adcSettingsDelta = -50;
        hbpOut.requestRead( HoverboardAPI::Codes::adcSettings );
        break;

      case GO_DISPLAY::MENU_WIRESHARKINC:
        hbpoutSetupADCsettings();
        adcSettingsDelta = 50;
        hbpOut.requestRead( HoverboardAPI::Codes::adcSettings );
        break;

      case GO_DISPLAY::MENU_WRITEFLASH:
      {
        uint16_t magic = 1238; // Magic number for flash;
        hbpOut.sendRawData( PROTOCOL_CMD_WRITEVAL, (unsigned char)HoverboardAPI::Codes::flashMagic, (unsigned char*) &magic, sizeof(magic), PROTOCOL_SOM_ACK );
        state = OD_LCD_MONITORINIT;
        break;
      }

      case GO_DISPLAY::MENU_PADDLEREAD:
        hbpOut.updateParamVariable( HoverboardAPI::Codes::paddleParameters, &paddelec.cfgPaddle, sizeof(paddelec.cfgPaddle) );
        hbpOut.requestRead( HoverboardAPI::Codes::paddleParameters );
        break;

      case GO_DISPLAY::MENU_PADDLEDRAGINC:
        if(paddelec.cfgPaddle.maxValidGyro != 0)        // Check if value were received.
        {
          paddelec.cfgPaddle.drag += 0.00005;
          hbpOut.sendRawData( PROTOCOL_CMD_WRITEVAL, (unsigned char)HoverboardAPI::Codes::paddleParameters, (unsigned char*) &paddelec.cfgPaddle, sizeof(paddelec.cfgPaddle), PROTOCOL_SOM_ACK );
        }
        else state = OD_LCD_MONITORINIT;
        break;

      case GO_DISPLAY::MENU_PADDLEDRAGDEC:
        if(paddelec.cfgPaddle.maxValidGyro != 0)        // Check if value were received.
        {
          paddelec.cfgPaddle.drag -= 0.00005;
          hbpOut.sendRawData( PROTOCOL_CMD_WRITEVAL, (unsigned char)HoverboardAPI::Codes::paddleParameters, (unsigned char*) &paddelec.cfgPaddle, sizeof(paddelec.cfgPaddle), PROTOCOL_SOM_ACK );
        }
        else state = OD_LCD_MONITORINIT;
        break;

      case GO_DISPLAY::MENU_PADDLEMULTINC:
        if(paddelec.cfgPaddle.maxValidGyro != 0)        // Check if value were received.
        {
          paddelec.cfgPaddle.pwmMultiplier += 0.005;
          hbpOut.sendRawData( PROTOCOL_CMD_WRITEVAL, (unsigned char)HoverboardAPI::Codes::paddleParameters, (unsigned char*) &paddelec.cfgPaddle, sizeof(paddelec.cfgPaddle), PROTOCOL_SOM_ACK );
        }
        else state = OD_LCD_MONITORINIT;
        break;

      case GO_DISPLAY::MENU_PADDLEMULTDEC:
        if(paddelec.cfgPaddle.maxValidGyro != 0)        // Check if value were received.
        {
          paddelec.cfgPaddle.pwmMultiplier -= 0.005;
          hbpOut.sendRawData( PROTOCOL_CMD_WRITEVAL, (unsigned char)HoverboardAPI::Codes::paddleParameters, (unsigned char*) &paddelec.cfgPaddle, sizeof(paddelec.cfgPaddle), PROTOCOL_SOM_ACK );
        }
        else state = OD_LCD_MONITORINIT;
        break;

      case GO_DISPLAY::MENU_PADDLESPEEDINC:
        if(paddelec.cfgPaddle.maxValidGyro != 0)        // Check if value were received.
        {
          paddelec.cfgPaddle.deltaRtoSpeed += 0.00005;
          hbpOut.sendRawData( PROTOCOL_CMD_WRITEVAL, (unsigned char)HoverboardAPI::Codes::paddleParameters, (unsigned char*) &paddelec.cfgPaddle, sizeof(paddelec.cfgPaddle), PROTOCOL_SOM_ACK );
        }
        else state = OD_LCD_MONITORINIT;
        break;

      case GO_DISPLAY::MENU_PADDLESPEEDDEC:
        if(paddelec.cfgPaddle.maxValidGyro != 0)        // Check if value were received.
        {
          paddelec.cfgPaddle.deltaRtoSpeed -= 0.00005;
          hbpOut.sendRawData( PROTOCOL_CMD_WRITEVAL, (unsigned char)HoverboardAPI::Codes::paddleParameters, (unsigned char*) &paddelec.cfgPaddle, sizeof(paddelec.cfgPaddle), PROTOCOL_SOM_ACK );
        }
        else state = OD_LCD_MONITORINIT;
        break;

      case GO_DISPLAY::MENU_PWM_VOLLGAS:
        PWMData.speed_max_power = 1000;
        hbpOut.sendPWMData(PWMData.pwm[0], PWMData.pwm[1], PWMData.speed_max_power, PWMData.speed_min_power, PWMData.speed_minimum_pwm, PROTOCOL_SOM_ACK);
        state = OD_LCD_MONITORINIT;
        break;

      case GO_DISPLAY::MENU_PWM_HALBGAS:
        PWMData.speed_max_power = 400;
        hbpOut.sendPWMData(PWMData.pwm[0], PWMData.pwm[1], PWMData.speed_max_power, PWMData.speed_min_power, PWMData.speed_minimum_pwm, PROTOCOL_SOM_ACK);
        state = OD_LCD_MONITORINIT;
        break;


      default:
        break;
      }

      if(BtnMenuJustPressed) state = OD_LCD_MONITORINIT;  // Go back
      break;


  default:
    state = OD_LCD_MONITORINIT;
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
  if( sysconfig.chan_out == COMM_CHAN_UART )
    pollUART();

  if( sysconfig.chan_out == COMM_CHAN_UDP || sysconfig.chan_in == COMM_CHAN_UDP )
    pollUDP();

  hbpOut.protocolTick();

  if( sysconfig.chan_in == COMM_CHAN_ESPNOW || sysconfig.chan_in == COMM_CHAN_UDP )
    hbpIn.protocolTick();
}



void setupCommunication()
{

  initializeOdroidGo();

  switch (sysconfig.chan_out)
  {
  case COMM_CHAN_ESPNOW:
    initializeESPnow();
    hbpOut.setSendSerialData(espSendDataWrapper);
    break;

  case COMM_CHAN_UDP:
    initializeUDP();
    hbpOut.setSendSerialData(udpSendDataWrapper);
    break;

  default:
    hbpOut.setSendSerialData(serialWriteWrapper);
    break;
  }

  switch (sysconfig.chan_in)
  {
  case COMM_CHAN_ESPNOW:
    initializeESPnow();
    setupRelaying();
    hbpIn.setSendSerialData(espSendDataWrapper);
    break;

  case COMM_CHAN_UDP:
    initializeUDP();
    setupRelaying();
    hbpIn.setSendSerialData(udpSendDataWrapper);
    break;

  default:
    hbpoutSetupHandlers();
#ifndef ODROID_GO_HW
    hbpoutSetupPWMtransmission();
    hbpoutScheduleReadings();
#endif
    break;
  }

  if(sysconfig.input == SYSCONF_IN_PADDLEIMU)
  {
    hbpOut.updateParamVariable( HoverboardAPI::Codes::paddleParameters, &paddelec.cfgPaddle, sizeof(paddelec.cfgPaddle) );
    hbpOut.updateParamHandler(  HoverboardAPI::Codes::paddleParameters, fn_defaultProcessing);
  }
}

void loopCommunication( void *pvparameters )
{
  while(1)
  {
    processOdroidGo();
    printProtocolMeasurements();
    processBuzzer();

    unsigned long start = millis();
    do
    {
      receiveAndprocessProtocol();
      delayMicroseconds(100);
    }
    while( millis() < start + MOTORINPUT_PERIOD );
  }
}
