#include <Arduino.h>
#include "config.h"
#include <esp_wifi.h>
#include <WiFi.h>
#include <crc.h>

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
#endif // PROTOCOL_TDP


uint8_t buf1[NUM_COM][bufferSize];
uint16_t i1[NUM_COM]={0,0,0};

uint8_t buf2[NUM_COM][bufferSize];
uint16_t i2[NUM_COM]={0,0,0};

uint8_t BTbuf[bufferSize];
uint16_t iBT =0;


struct motorControl {
  int16_t steer;
  int16_t pwm;      // called "speed" in hoverboard firmware, but is only pwm duty cycle in promille
                    // Values from -1000 to 1000. Negative values represent driving backwards.
};

motorControl motor = {0,0};
int16_t actualSpeed_mh = 0;  // motor speed in m/h
int16_t actualSteer_mh    = 0;  // motor steer


#ifdef PADDELEC
  #include <Paddelec.h>
  Paddelec paddelec = Paddelec();
#endif // PADDELEC

#ifdef NUNCHUCK
  #include <ArduinoNunchuk.h>
  ArduinoNunchuk nunchuk = ArduinoNunchuk();
#endif // NUNCHUCK

unsigned long nextMillisMotorInput = 0;

void setup() {

  #ifdef SERIAL2_GNDPIN
    pinMode(SERIAL2_GNDPIN,OUTPUT);
    digitalWrite(SERIAL2_GNDPIN,LOW);
  #endif
  #ifdef SERIAL2_VCCPIN
    pinMode(SERIAL2_VCCPIN,OUTPUT);
    digitalWrite(SERIAL2_VCCPIN,HIGH);
  #endif
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
#endif // BLUETOOTH 
        i2[num] = 0;
      }
    }    
  }
}

int limit(int min, int value, int max) 
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
#define SPEED_PWM_CONVERSION_FACTOR 20.0   // Assume 100% PWM = 1000 = Full Speed = 20km/h = 20000 m/h. Therefore 20000 / 1000 = 20
#define SPEED_FILTER                 0.1   // Low pass Filter Value. 1 means no filter at all, 0 no value update.
void updateSpeed()
{
  actualSpeed_mh = (int16_t) (actualSpeed_mh * (1.0 - SPEED_FILTER) + motor.pwm   * SPEED_FILTER * SPEED_PWM_CONVERSION_FACTOR);
  actualSteer_mh = (int16_t) (actualSteer_mh * (1.0 - SPEED_FILTER) + motor.steer * SPEED_FILTER * SPEED_PWM_CONVERSION_FACTOR);
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
    paddelec.update(motor.pwm, motor.steer, actualSpeed_mh, actualSteer_mh);
    if(debug)
    {
      paddelec.debug(*COM[DEBUG_COM]);
      for(byte cln = 0; cln < MAX_NMEA_CLIENTS; cln++)
        if(TCPClient[1][cln]) paddelec.debug(TCPClient[1][cln]); 
    }
#endif // PADDELEC

#if defined(NUNCHUCK) && defined(PADDELEC)
    if(paddelec.gametrak1.r < 400 || paddelec.gametrak2.r < 400) // switch to nunchuck control when paddle is not used
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
    motor.pwm = limit(-1000, motor.pwm, 1000);  
    motor.steer = limit(-1000, motor.steer, 1000);  

    /* calc checksum */
    uint32_t crc;
    crc = 0;
    crc32((const void *)&motor, 4, &crc); // 4 2x uint16_t = 4 bytes

    /* Send motor pwm values to motor control unit */
    COM[MOTOR_COM]->write((uint8_t *) &motor.steer, sizeof(motor.steer)); 
    COM[MOTOR_COM]->write((uint8_t *) &motor.pwm, sizeof(motor.pwm));
    COM[MOTOR_COM]->write((uint8_t *) &crc, sizeof(crc));
    updateSpeed();

    // debug output
    for(byte cln = 0; cln < MAX_NMEA_CLIENTS; cln++)
    {   
      if(TCPClient[1][cln]) {                    
        if(debug) TCPClient[1][cln].print(" U: ");
        if(debug) TCPClient[1][cln].printf("%8i %8i\n", motor.pwm, motor.steer);
      } 
    }
    if(debug) COM[DEBUG_COM]->print(" U: ");
    if(debug) COM[DEBUG_COM]->printf("%8i %8i %8i", motor.pwm, motor.steer, crc);
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

