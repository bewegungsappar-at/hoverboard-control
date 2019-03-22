#include <Arduino.h>
#include "config.h"
#include "main.h"
#include "input.h"
#include "output.h"
#include "serialbridge.h"
#include "ArduinoNunchuk.h"

#ifdef MULTITASKING
  TaskHandle_t TaskMainloop, TaskMotorcommunication;
#endif //MULTITASKING


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
  setupOutput();
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

#ifdef MULTITASKING
  xTaskCreatePinnedToCore(
    mainloop,                 // Task function.
    "Main_loop",              // name of task.
    4000,                     // Stack size of task
    (void *)1,                // parameter of the task
    1,                        // priority of the task
    &TaskMainloop,            // Task handle to keep track of created task
    1);                       // Core (0 is used by ESP32 connectivity)

  xTaskCreatePinnedToCore(
    motorCommunication,       // Task function.
    "Motor_Comm",             // name of task.
    4000,                     // Stack size of task
    (void *)1,                // parameter of the task
    1,                        // priority of the task
    &TaskMotorcommunication,  // Task handle to keep track of created task
    0);                       // Core (0 is used by ESP32 connectivity)
#endif //MULTITASKING
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


  #ifdef MULTITASKING

    delay(5);

  #else

    mainloop( (void *)1 );
    motorCommunication( (void *)1 );
    delay(MOTORINPUT_PERIOD);

  #endif //MULTITASKING

}
