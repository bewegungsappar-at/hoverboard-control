#pragma once

#include <Arduino.h>

void setupInput();
void loopInput( void *pvparameters );

typedef enum
{
    NEEDCALIB,
    IDLE,
    SETUP,
    RUNNING,
    RELEASE
} nunchuk_state;

extern nunchuk_state nunchukState;

#include "Paddelec.h"
extern Paddelec paddelec;