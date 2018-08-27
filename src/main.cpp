// ESP32 WiFi <-> 3x UART Bridge
// by AlphaLima
// www.LK8000.com

// Disclaimer: Don't use  for life support systems
// or any other situations where system failure may affect
// user or environmental safety.

#include <Arduino.h>
#include "config.h"
#include <esp_wifi.h>
#include <WiFi.h>


#include <WebServer.h>

WebServer wupserver(80);

#ifdef BLUETOOTH
#include <BluetoothSerial.h>
BluetoothSerial SerialBT; 
#endif

#ifdef OTA_HANDLER  
#include <ArduinoOTA.h> 

#endif // OTA_HANDLER

//HardwareSerial Serial1(1); // already defined in HardwareSerial.h - Hardware serial library for Wiring
//HardwareSerial Serial2(2); // already defined in HardwareSerial.h - Hardware serial library for Wiring
HardwareSerial* COM[NUM_COM] = {&Serial, &Serial1 , &Serial2};

#define MAX_NMEA_CLIENTS 4
#ifdef PROTOCOL_TCP
#include <WiFiClient.h>
WiFiServer server_0(SERIAL0_TCP_PORT);
WiFiServer server_1(SERIAL1_TCP_PORT);
WiFiServer server_2(SERIAL2_TCP_PORT);
WiFiServer *server[NUM_COM]={&server_0,&server_1,&server_2};
WiFiClient TCPClient[NUM_COM][MAX_NMEA_CLIENTS];
#endif

#ifdef PROTOCOL_UDP
#include <WiFiUdp.h>
WiFiUDP udp;
IPAddress remoteIp;
#endif


uint8_t buf1[NUM_COM][bufferSize];
uint16_t i1[NUM_COM]={0,0,0};

uint8_t buf2[NUM_COM][bufferSize];
uint16_t i2[NUM_COM]={0,0,0};

uint8_t BTbuf[bufferSize];
uint16_t iBT =0;

struct Gametrak {   // spherical coordinates
  uint16_t r;
  uint16_t r_last;
  uint16_t theta;
  uint16_t phi;
};

Gametrak gametrak1 = {0,0,0,0};
Gametrak gametrak2 = {0,0,0,0};

struct motorControl {
  int16_t steer;
  int16_t speed;
};

motorControl motor = {0,0};

void setup() {

  delay(500);
  
  COM[0]->begin(UART_BAUD0, SERIAL_PARAM0, SERIAL0_RXPIN, SERIAL0_TXPIN);
  COM[1]->begin(UART_BAUD1, SERIAL_PARAM1, SERIAL1_RXPIN, SERIAL1_TXPIN);
  COM[2]->begin(UART_BAUD2, SERIAL_PARAM2, SERIAL2_RXPIN, SERIAL2_TXPIN);
  
  // STATION mode (ESP connects to router and gets an IP)
  // Assuming phone is also connected to that router
  // from RoboRemo you must connect to the IP of the ESP
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pw);
  if(debug) COM[DEBUG_COM]->print("try to Connect to Wireless network: ");
  if(debug) COM[DEBUG_COM]->println(ssid);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    delay(500);
    if(debug) COM[DEBUG_COM]->println("Connection Failed! Fallback to AP Mode");
    //AP mode (phone connects directly to ESP) (no router)
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(ip, ip, netmask); // configure ip address for softAP 
    WiFi.softAP(ssid_AP, pw_AP); // configure ssid and password for softAP
    if(debug) COM[DEBUG_COM]->print("SSID ");
    if(debug) COM[DEBUG_COM]->print(ssid_AP);
    if(debug) COM[DEBUG_COM]->print(" pw ");
    if(debug) COM[DEBUG_COM]->println(pw_AP);
    break;
  }
  if(debug) COM[DEBUG_COM]->println("WiFi ready");
  
#ifdef BLUETOOTH
  if(debug) COM[DEBUG_COM]->println("Open Bluetooth Server");  
  SerialBT.begin(ssid); //Bluetooth device name
 #endif
#ifdef OTA_HANDLER  
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
  // if DNSServer is started with "*" for domain name, it will reply with
  // provided IP to all DNS request

  ArduinoOTA.begin();
#endif // OTA_HANDLER    

  #ifdef PROTOCOL_TCP
  if(debug) COM[DEBUG_COM]->println("Starting TCP Server 1");  
  server[0]->begin(); // start TCP server 
  server[0]->setNoDelay(true);
  COM[1]->println("Starting TCP Server 2");
  if(debug) COM[DEBUG_COM]->println("Starting TCP Server 2");  
  server[1]->begin(); // start TCP server 
  server[1]->setNoDelay(true);
  COM[2]->println("Starting TCP Server 3");
  if(debug) COM[DEBUG_COM]->println("Starting TCP Server 3");  
  server[2]->begin(); // start TCP server   
  server[2]->setNoDelay(true);
  #endif

  #ifdef PROTOCOL_UDP
  if(debug) COM[DEBUG_COM]->println("Starting UDP Server 1");
  udp.begin(SERIAL0_TCP_PORT); // start UDP server 

  if(debug) COM[DEBUG_COM]->println("Starting UDP Server 2");
  udp.begin(SERIAL1_TCP_PORT); // start UDP server 

  if(debug) COM[DEBUG_COM]->println("Starting UDP Server 3");
  udp.begin(SERIAL2_TCP_PORT); // start UDP server      
  #endif

  esp_err_t esp_wifi_set_max_tx_power(50);  //lower WiFi Power
}

void readGametraks() {
  gametrak1.r_last = gametrak1.r;
  gametrak1.r      = analogRead(GAMETRAK1_RPIN);
  gametrak1.phi    = analogRead(GAMETRAK1_PHIPIN);
  gametrak1.theta  = analogRead(GAMETRAK1_THETAPIN);

  gametrak2.r_last = gametrak2.r;
  gametrak2.r      = analogRead(GAMETRAK2_RPIN);
  gametrak2.phi    = 4096 - analogRead(GAMETRAK2_PHIPIN);
  gametrak2.theta  = 4096 - analogRead(GAMETRAK2_THETAPIN);

  if(debug) COM[DEBUG_COM]->print("Gametraks ");
  if(debug) COM[DEBUG_COM]->printf("%4u %4u %4u | ", gametrak1.r, gametrak1.phi, gametrak1.theta);
  if(debug) COM[DEBUG_COM]->printf("%4u %4u %4u\n",  gametrak2.r, gametrak2.phi, gametrak2.theta);

  if(debug) TCPClient[0][0].print("Gametraks ");
  if(debug) TCPClient[0][0].printf("%4u %4u %4u | ", gametrak1.r, gametrak1.phi, gametrak1.theta);
  if(debug) TCPClient[0][0].printf("%4u %4u %4u\n",  gametrak2.r, gametrak2.phi, gametrak2.theta);

}

void loop() 
{  
#ifdef OTA_HANDLER  
  ArduinoOTA.handle();
#endif // OTA_HANDLER

#ifdef BLUETOOTH
  // receive from Bluetooth:
  if(SerialBT.hasClient()) 
  {
    while(SerialBT.available())
    {
      BTbuf[iBT] = SerialBT.read(); // read char from client 
      if(iBT <bufferSize-1) iBT++;
    }          
    for(int num= 0; num < NUM_COM ; num++)
      COM[num]->write(BTbuf,iBT); // now send to UART(num):          
    iBT = 0;
  }  
#endif  
#ifdef PROTOCOL_TCP
  for(int num= 0; num < NUM_COM ; num++)
  {
    if (server[num]->hasClient())
    {
      for(byte i = 0; i < MAX_NMEA_CLIENTS; i++){
        //find free/disconnected spot
        if (!TCPClient[num][i] || !TCPClient[num][i].connected()){
          if(TCPClient[num][i]) TCPClient[num][i].stop();
          TCPClient[num][i] = server[num]->available();
          if(debug) COM[DEBUG_COM]->print("New client for COM"); 
          if(debug) COM[DEBUG_COM]->print(num); 
          if(debug) COM[DEBUG_COM]->println(i);
          continue;
        }
      }
      //no free/disconnected spot so reject
      WiFiClient TmpserverClient = server[num]->available();
      TmpserverClient.stop();
    }
  }
#endif
 
  for(int num= 0; num < NUM_COM ; num++)
  {
    if(COM[num] != NULL)          
    {
      for(byte cln = 0; cln < MAX_NMEA_CLIENTS; cln++)
      {               
        if(TCPClient[num][cln]) 
        {
          while(TCPClient[num][cln].available())
          {
            buf1[num][i1[num]] = TCPClient[num][cln].read(); // read char from client 
            if(i1[num]<bufferSize-1) i1[num]++;
          } 

          COM[num]->write(buf1[num], i1[num]); // now send to UART(num):
          i1[num] = 0;
        }
      }
  
      if(COM[num]->available())
      {
        while(COM[num]->available())
        {     
          buf2[num][i2[num]] = COM[num]->read(); // read char from UART(num)
          if(i2[num]<bufferSize-1) i2[num]++;
        }
        // now send to WiFi:
        for(byte cln = 0; cln < MAX_NMEA_CLIENTS; cln++)
        {   
          if(TCPClient[num][cln])                     
            TCPClient[num][cln].write(buf2[num], i2[num]);
        }
#ifdef BLUETOOTH        
        // now send to Bluetooth:
        if(SerialBT.hasClient())      
          SerialBT.write(buf2[num], i2[num]);        
        i2[num] = 0;
#endif        
      }
    }    
  }
  readGametraks();



  if(gametrak1.phi-gametrak2.phi < -1000) {
    // gametrak2 side of paddel is down
    motor.speed = (gametrak1.r - gametrak1.r_last) * 10;
    motor.steer = motor.speed * 0.5;
  } else if(gametrak1.phi-gametrak2.phi > 1000) {
    // gametrak1 side of paddel is down
    motor.speed = (gametrak2.r - gametrak2.r_last) * 10;
    motor.steer = motor.speed * -0.5;
  } else {
    // paddel is not engaged
    // decelerate slowly
    if(motor.speed < -1) {
      motor.speed = motor.speed + 1;
    } else if(motor.speed > 1) {
      motor.speed = motor.speed - 1;
    } else {
      motor.speed = 0;
    }
    motor.steer = 0;
  }
  
  /* limit motor speed */
  if(motor.speed < -500) {
    motor.speed = -500;
  } else if(motor.speed > 500) {
    motor.speed = 500;
  } 
  
  /* limit steering */
  if(motor.steer < -100) {
    motor.steer = -100;
  } else if(motor.steer > 100) {
    motor.steer = 100;
  } 
    
  COM[MOTOR_COM]->write((uint8_t *) &motor.steer, sizeof(motor.steer)); 
  COM[MOTOR_COM]->write((uint8_t *) &motor.speed, sizeof(motor.speed));

  for(byte cln = 0; cln < MAX_NMEA_CLIENTS; cln++)
  {   
    if(TCPClient[1][cln]) {                    
      if(debug) TCPClient[1][cln].print("UART: ");
      if(debug) TCPClient[1][cln].printf("%8i %8i %8i %8i %8i\n", motor.speed, motor.steer, gametrak1.r - gametrak1.r_last, gametrak2.r - gametrak2.r_last, gametrak1.phi-gametrak2.phi);
    }
  } 
  delay(20); 

}

