#pragma once

#include "config.h"
#include <Arduino.h>


void setupOTA();
void ota();

extern HardwareSerial* COM[NUM_COM];

void setupSerial();
