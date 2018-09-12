#ifndef SERIALBRIDGE_H
#define SERIALBRIDGE_H


#include "config.h"
#include <Arduino.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiClient.h>

extern HardwareSerial* COM[NUM_COM];

extern WiFiServer server_0;
extern WiFiServer server_1;
extern WiFiServer server_2;
extern WiFiServer *server[NUM_COM];
extern WiFiClient TCPClient[NUM_COM][MAX_NMEA_CLIENTS];

extern uint8_t buf1[NUM_COM][bufferSize];
extern uint16_t i1[NUM_COM];

extern uint8_t buf2[NUM_COM][bufferSize];
extern uint16_t i2[NUM_COM];

void setupSerial();
void setupWifi();
void setupSerialbridge();
void setupOTA();
void bridge();

#endif
