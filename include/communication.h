#pragma once

#include "config.h"
#include <Arduino.h>

void setupCommunication();
void loopCommunication( void *pvparameters );
void updateSpeed();
