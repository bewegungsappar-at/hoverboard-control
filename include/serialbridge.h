#pragma once

#include "config.h"
#include <Arduino.h>


#ifdef OTA_HANDLER
    void setupOTA();
    void ota();
#endif

extern HardwareSerial* COM[NUM_COM];

void setupSerial();
