#include <Arduino.h>
#include "config.h"
#include "main.h"
#include "input.h"
#include "output.h"
#include "serialbridge.h"

#ifdef MULTITASKING
TaskHandle_t TaskMainloop, TaskMotorcommunication;
void mainloop(void *pvParameters);
void motorCommunication(void *pvParameters);
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
#endif

motorControl motor = { {0.0, 0.0} , {0.0, 0.0} };
int32_t deltaMillis;

void setup() {
  setupOutput();
  setupInput();
  setupSerial();

#ifdef WIFI
  setupWifi();
  setupSerialbridge();
#endif

#ifdef DEBUG_PLOTTER
  // Start plotter
 // plot.Begin();

  plot.AddTimeGraph( "Motor Output", 500, "PWM", motor.setpoint.pwm, "Steer", motor.setpoint.steer, "vehicle Speed", motor.measured.actualSpeed_kmh, "vehicle Steer", motor.measured.actualSteer_kmh );
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
    plot.Plot();
  #endif


  #ifdef MULTITASKING

    delay(20);

  #else

    mainloop( (void *)1 );
    motorCommunication( (void *)1 );
    delay(MOTORINPUT_PERIOD);

  #endif //MULTITASKING

}
