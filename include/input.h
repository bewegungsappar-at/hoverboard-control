#pragma once

#include "config.h"
#include <Arduino.h>

void setupInput();
void loopInput( void *pvparameters );

extern volatile int espnowTimeout;