#pragma once

#include <Arduino.h>

void setupCommunication();
void loopCommunication( void *pvparameters );
void hbpoutSetupPWMtransmission();

extern double odroidSpeed;
extern double odroidSteer;