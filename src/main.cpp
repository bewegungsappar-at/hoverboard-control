#include <Arduino.h>
#include "config.h"
#include "main.h"
#include "input.h"
#include "communication.h"
#include "serialbridge.h"
#include "ArduinoNunchuk.h"

  TaskHandle_t TaskInput, TaskCommunication;


#ifdef DEBUG_CONSOLE
  bool debug = true;
#else
  bool debug = false;
#endif

volatile motorControl motor = { {0.0, 0.0} , {0.0, 0.0} };
volatile int32_t deltaMillis;

void setup() {

  setupSerial();
  setupCommunication();
  setupInput();

  setupOTA();

  xTaskCreatePinnedToCore(
    loopInput,                 // Task function.
    "loopInput",              // name of task.
    4000,                     // Stack size of task
    (void *)1,                // parameter of the task
    1,                        // priority of the task
    &TaskInput,            // Task handle to keep track of created task
    1);                       // Core (0 is used by ESP32 connectivity)

  xTaskCreatePinnedToCore(
    loopCommunication,       // Task function.
    "loopCommunication",             // name of task.
    4000,                     // Stack size of task
    (void *)1,                // parameter of the task
    1,                        // priority of the task
    &TaskCommunication,  // Task handle to keep track of created task
    1);                       // Core (0 is used by ESP32 connectivity)
}


void loop() {

    ota();

  delay(5);
}

// Incrementally decrease variable
void slowReset(volatile double &variable, double goal, double step, double fact) {
  variable  += (goal - variable) * fact;
  if      ((variable - goal) > step) variable -= step;
  else if ((goal - variable) > step) variable += step;
  else                               variable  = goal;
}

void slowReset(int32_t &variable, double goal, int32_t step, double fact) {
  variable  += (int32_t)((goal - variable) * fact);
  if      ((variable - goal) > step) variable -= step;
  else if ((goal - variable) > step) variable += step;
  else                               variable  = goal;
}