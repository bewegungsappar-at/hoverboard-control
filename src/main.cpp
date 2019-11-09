#include <Arduino.h>
#include "config.h"
#include "main.h"
#include "input.h"
#include "communication.h"
#include "serialbridge.h"
#include "ArduinoNunchuk.h"

  TaskHandle_t TaskInput, TaskCommunication;


#ifdef DEBUG_OLED
  #include "oled.h"
#endif

#ifdef DEBUG_CONSOLE
  bool debug = true;
#else
  bool debug = false;
#endif

#ifdef DEBUG_PLOTTER
  #include <Plotter.h>
  Plotter plot;
  double plotterTempDouble[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
#endif

volatile motorControl motor = { {0.0, 0.0} , {0.0, 0.0} };
volatile int32_t deltaMillis;

void setup() {

#ifdef DEBUG_OLED
  setupOLED();
#endif

  setupSerial();
  setupCommunication();
  setupInput();

#ifdef WIFI
  setupWifi();
  setupSerialbridge();
#endif

#ifdef DEBUG_PLOTTER
  plot.AddTimeGraph( "Temp1", 500, "0", plotterTempDouble[0], "1", plotterTempDouble[1] );
  plot.AddTimeGraph( "Temp2", 500, "2", plotterTempDouble[2], "3", plotterTempDouble[3] );
  plot.AddTimeGraph( "Temp3", 500, "4", plotterTempDouble[4], "5", plotterTempDouble[5] );
#endif

#ifdef OTA_HANDLER
  setupOTA();
#endif

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

  #ifdef OTA_HANDLER
    ota();
  #endif // OTA_HANDLER

  #ifdef WIFI
    bridge();
  #endif

  #ifdef DEBUG_PLOTTER
//    plot.Plot();
  #endif


  delay(5);
}
