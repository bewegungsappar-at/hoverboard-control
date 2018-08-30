#include <Arduino.h>
#include "config.h"
#include <esp_wifi.h>
#include <WiFi.h>
#include <CRC32.h>

#ifdef BLUETOOTH
  #include <BluetoothSerial.h>
  BluetoothSerial SerialBT; 
#endif // BLUETOOTH

#ifdef OTA_HANDLER  
  #include <ArduinoOTA.h> 
#endif // OTA_HANDLER

HardwareSerial* COM[NUM_COM] = {&Serial, &Serial1 , &Serial2};

#define MAX_NMEA_CLIENTS 4
#ifdef PROTOCOL_TCP
  #include <WiFiClient.h>
  WiFiServer server_0(SERIAL0_TCP_PORT);
  WiFiServer server_1(SERIAL1_TCP_PORT);
  WiFiServer server_2(SERIAL2_TCP_PORT);
  WiFiServer *server[NUM_COM]={&server_0,&server_1,&server_2};
  WiFiClient TCPClient[NUM_COM][MAX_NMEA_CLIENTS];
#endif // PROTOCOL_UDP

#ifdef PROTOCOL_UDP
  #include <WiFiUdp.h>
  WiFiUDP udp;
  IPAddress remoteIp;
#endif // PROTOCOL_UDP


uint8_t buf1[NUM_COM][bufferSize];
uint16_t i1[NUM_COM]={0,0,0};

uint8_t buf2[NUM_COM][bufferSize];
uint16_t i2[NUM_COM]={0,0,0};

uint8_t BTbuf[bufferSize];
uint16_t iBT =0;


struct motorControl {
  int16_t steer;
  int16_t speed;
};

motorControl motor = {0,0};

#ifdef PADDELEC
  #include <Paddelec.h>
  Paddelec paddelec = Paddelec(*COM[DEBUG_COM]);
#endif // PADDELEC

#ifdef NUNCHUCK
  #include <ArduinoNunchuk.h>
  ArduinoNunchuk nunchuk = ArduinoNunchuk();
#endif // NUNCHUCK

unsigned long nextMillisMotorInput = 0;

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
    delay(200);
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
#endif // BLUETOOTH

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
#endif // PROTOCOL_TCP

#ifdef PROTOCOL_UDP
  if(debug) COM[DEBUG_COM]->println("Starting UDP Server 1");
  udp.begin(SERIAL0_TCP_PORT); // start UDP server 

  if(debug) COM[DEBUG_COM]->println("Starting UDP Server 2");
  udp.begin(SERIAL1_TCP_PORT); // start UDP server 

  if(debug) COM[DEBUG_COM]->println("Starting UDP Server 3");
  udp.begin(SERIAL2_TCP_PORT); // start UDP server      
#endif // PROTOCOL_UDP

#ifdef NUNCHUCK
  nunchuk.init();
#endif // NUNCHUCK
  /* Keep this at the end of setup */
  nextMillisMotorInput = millis();
}

#ifdef BLUETOOTH
void bridgeBT() 
{
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
}
#endif // BLUETOOTH

#ifdef PROTOCOL_TCP
void bridgeTCP()
{
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
}
#endif // PROTOCOL_TCP

void bridge()
{
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
#endif // BLUETOOTH 
      }
    }    
  }
}

void loop() 
{  
#ifdef OTA_HANDLER  
  ArduinoOTA.handle();
#endif // OTA_HANDLER

#ifdef BLUETOOTH
  bridgeBT();
#endif // BLUETOOTH

#ifdef PROTOCOL_TCP
  bridgeTCP();
#endif // PROTOCOL_TCP

  bridge();



  long deltaMillis = millis() - nextMillisMotorInput;
  if(deltaMillis >= 0)
  {
    nextMillisMotorInput += MOTORINPUT_PERIOD;
#ifdef PADDELEC
    paddelec.update(motor.speed, motor.steer);
    if(debug)
    {
      paddelec.debug(*COM[DEBUG_COM]);
      for(byte cln = 0; cln < MAX_NMEA_CLIENTS; cln++)
        if(TCPClient[1][cln]) paddelec.debug(TCPClient[1][cln]); 
    }

  #if defined(NUNCHUCK)
    if(paddelec.gametrak1.r < 400 || paddelec.gametrak2.r < 400) // switch to nunchuck control when paddle is not used
    {
      nunchuk.update(motor.speed, motor.steer);
      if(debug) nunchuk.debug(*COM[DEBUG_COM]);
    }
  #endif // NUNCHUCK
#endif // PADDELEC

#if defined(NUNCHUCK) && !defined(PADDELEC) 
      nunchuk.update(motor.speed, motor.steer);
      nunchuk.debug(*COM[DEBUG_COM]);
#endif // NUNCHUCK

    /* Send motor speed values to motor control unit */
    CRC32 crc;
    crc.update(&motor.steer, sizeof(motor.steer));
    COM[MOTOR_COM]->write((uint8_t *) &motor.steer, sizeof(motor.steer)); 
    crc.update(&motor.speed, sizeof(motor.speed));
    COM[MOTOR_COM]->write((uint8_t *) &motor.speed, sizeof(motor.speed));
    uint32_t checksum = crc.finalize();
    COM[MOTOR_COM]->write((uint8_t *) &checksum, sizeof(checksum));
    // debug output
    for(byte cln = 0; cln < MAX_NMEA_CLIENTS; cln++)
    {   
      if(TCPClient[1][cln]) {                    
        if(debug) TCPClient[1][cln].print(" U: ");
        if(debug) TCPClient[1][cln].printf("%8i %8i\n", motor.speed, motor.steer);
      } 
    }
    if(debug) COM[DEBUG_COM]->print(" U: ");
    if(debug) COM[DEBUG_COM]->printf("%8i %8i", motor.speed, motor.steer);
    if(debug) COM[DEBUG_COM]->println();
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

