#include "serialbridge.h"
#include <Arduino.h>
#include "config.h"
#include "main.h"

#include <ArduinoOTA.h>


HardwareSerial* COM[NUM_COM] = {&Serial, &Serial1 , &Serial2};


void setupSerial() {
  #ifdef SERIAL2_GNDPIN
    pinMode(SERIAL2_GNDPIN,OUTPUT);
    digitalWrite(SERIAL2_GNDPIN,LOW);
  #endif
  #ifdef SERIAL2_VCCPIN
    pinMode(SERIAL2_VCCPIN,OUTPUT);
    digitalWrite(SERIAL2_VCCPIN,HIGH);
  #endif
  delay(300);

    COM[0]->begin(UART_BAUD0, SERIAL_PARAM0, SERIAL0_RXPIN, SERIAL0_TXPIN);
    COM[1]->begin(UART_BAUD1, SERIAL_PARAM1, SERIAL1_RXPIN, SERIAL1_TXPIN);
    COM[2]->begin(UART_BAUD2, SERIAL_PARAM2, SERIAL2_RXPIN, SERIAL2_TXPIN);
}


void setupOTA() {
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
}
void ota() {
  ArduinoOTA.handle();
}