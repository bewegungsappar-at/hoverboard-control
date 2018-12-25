#pragma once

#include "config.h"
#include <Arduino.h>

#ifdef WIFI
    #include <esp_wifi.h>
    void setupWifi();
    void setupSerialbridge();
    void setupOTA();
    void bridge();
    void ota();
    extern uint8_t buf1[NUM_COM][bufferSize];
    extern uint16_t i1[NUM_COM];

    extern uint8_t buf2[NUM_COM][bufferSize];
    extern uint16_t i2[NUM_COM];
#endif


extern HardwareSerial* COM[NUM_COM];

void setupSerial();
