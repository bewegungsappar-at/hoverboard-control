#pragma once

#include "config.h"
#include <Arduino.h>

void setupOutput();
void motorCommunication( void * pvparameters);
void updateSpeed();

#ifdef OUTPUT_PROTOCOL
size_t send_serial_data( const uint8_t *data, size_t len );
#endif