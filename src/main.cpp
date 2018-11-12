#include <Arduino.h>
#include "config.h"
#include <crc.h>
#include "serialbridge.h"

#ifdef MULTITASKING
TaskHandle_t TaskMainloop, TaskMotorcommunication;
void mainloop(void *pvParameters);
void motorCommunication(void *pvParameters);
#endif //MULTITASKING

#ifdef OUTPUT_PROTOCOL
  #include <protocol.h>

  size_t send_serial_data( const uint8_t *data, size_t len ) {
    COM[DEBUG_COM]->write(data,len);
    COM[DEBUG_COM]->println();
    return COM[MOTOR_COM]->write(data,len);
  }
#endif

#if defined(INPUT_PADDELEC) || defined(INPUT_PADDELECIMU)
  #include "Paddelec.h"
  Paddelec paddelec = Paddelec();
#endif // INPUT_PADDELEC

#ifdef INPUT_NUNCHUCK
  #include <ArduinoNunchuk.h>
  ArduinoNunchuk nunchuk = ArduinoNunchuk();
#endif // INPUT_NUNCHUCK

#ifdef OUTPUT_ESPnowMASTER
  #include "ESPnowMaster.h"
#endif

#ifdef INPUT_ESPnowSLAVE
  #include "ESPnowSlave.h"
  bool ESPnowDataReceived = false;
#endif

#ifdef INPUT_PLATOONING
  #include "Platooning.h"
  Platooning platooning = Platooning();
#endif

#ifdef OLED
  #include "oled.h"
#endif

#ifdef INPUT_BLE
  #include "BLE.h"
#endif

#ifdef INPUT_IMU
  #include <IMU.h>
  Imu imu = Imu();
#endif

bool debug = false;

motorControl motor = {0.0, 0.0, 0.0, 0.0};
uint32_t millisMotorcomm = 0;      // virtual timer for motor update
int32_t deltaMillis;
int errorCount=0;

void setup() {

#ifdef SETDEBUG
  debug = true;
#endif

  setupSerial();

#ifdef WIFI
  setupWifi();
  setupSerialbridge();
#endif

#ifdef OUTPUT_ESPnowMASTER
  setupESPnowMaster();
#endif

#ifdef INPUT_ESPnowSLAVE
  setupESPnowSlave();
#endif

#ifdef OLED
  setupOLED();
#endif

#ifdef OTA_HANDLER  
  setupOTA();
#endif 

#ifdef INPUT_NUNCHUCK
  nunchuk.init();
#endif

#ifdef INPUT_BLE
  setupBLE();
#endif

#ifdef INPUT_IMU
  imu.init();
#endif 

#if defined(INPUT_PADDELEC) || defined(INPUT_PADDELECIMU)
  paddelec.init();
#endif //INPUT_PADDELEC


  /* Keep this at the end of setup */
  millisMotorcomm = millis();

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


double limit(double min, double value, double max) {
  if(value<min) value = min;
  if(value>max) value = max;
  return value;
}

// Incrementally decrease variable
void slowReset(double &variable, double goal, double step) {
  if      ((variable - goal) > step) variable -= step;
  else if ((goal - variable) > step) variable += step;
  else                               variable  = goal;
}

/* 
* Dummy function since no speed feedback from Motor control is implemented right now.
* For now, we just use pwm, some conversion factor and low pass filter as a model.
* Values are in m/h 
*/ 
#define SPEED_PWM_CONVERSION_FACTOR  0.2   // Assume 100% PWM = 1000 = Full Speed = 20km/h = 20000 m/h. Therefore 20000 / 1000 = 20
#define SPEED_FILTER                 0.015  // Low pass Filter Value. 1 means no filter at all, 0 no value update.
void updateSpeed() {
  motor.actualSpeed_kmh = motor.actualSpeed_kmh * (1.0 - (SPEED_FILTER * deltaMillis)) + motor.pwm   * (SPEED_FILTER * deltaMillis) * SPEED_PWM_CONVERSION_FACTOR;
  motor.actualSteer_kmh = motor.actualSteer_kmh * (1.0 - (SPEED_FILTER * deltaMillis)) + motor.steer * (SPEED_FILTER * deltaMillis) * SPEED_PWM_CONVERSION_FACTOR;
}

#ifdef OUTPUT_ESPnowMASTER 
  bool isPaired = false;
#endif

void loop() {  
#ifdef MULTITASKING
  // nope, do nothing here
  vTaskDelay(1000); // wait as much as posible ...
}

void mainloop( void *pvparameters ) {
    int taskno = (int)pvparameters;
    while(1) {
#endif //MULTITASKING

  #ifdef OTA_HANDLER  
    ota();
  #endif // OTA_HANDLER

  #ifdef WIFI
    bridge();
  #endif

    deltaMillis = millis() - millisMotorcomm;
    millisMotorcomm = millis();

  // Process all Inputs
  do {
    slowReset(motor.pwm, 0.0, 10.0);
    slowReset(motor.steer, 0.0, 10.0);

  #ifdef INPUT_ESPnowSLAVE
    // Disable all other Input Methods as soon as data from ESPnow was received
    if(ESPnowDataReceived) break;
  #endif

  #ifdef INPUT_BLE
    //  loopBLE();
    motor.pwm = ble_pitch;
    motor.steer = ble_roll;
    slowReset(ble_pitch, 0.0, 0.1);
    slowReset(ble_roll, 0.0, 0.1);
    break;
  #endif

  #ifdef INPUT_IMU
    //  imu.loopIMU();
    imu.update(motor.pwm, motor.steer);
    if(debug) imu.debug(*COM[DEBUG_COM]);
    
    // Allow other Inputs when no Button is pressed
    if(imu.cButton == 1) break;
  #endif

  #ifdef INPUT_PLATOONING
    platooning.update(motor.pwm, motor.steer);
    if(debug) platooning.debug(*COM[DEBUG_COM]);

    // Allow other Inputs when Gametrak is not pulled out 
    if(platooning.gametrak1.getR_mm() > platooning.cfgPlatooning.rActivationThreshold_mm) break;
  #endif

  #if defined(INPUT_NUNCHUCK)
    int nunchuckError = nunchuk.update(motor.pwm, motor.steer);
    ++errorCount;
    nunchuk.debug(*COM[DEBUG_COM]);
    if(nunchuckError >= 1000) {
      if(debug) COM[DEBUG_COM]->printf("Reinit Nunchuck %4i ", nunchuckError);
    } else if(nunchuckError >= 100) {
      if(debug) COM[DEBUG_COM]->printf("I2C Problems %4i ", nunchuckError);
    } else if(nunchuckError > 0) {
      if(debug) COM[DEBUG_COM]->printf("Nunchuck Comm Problems %4i ", nunchuckError);
    } else  {
      errorCount = 0;
    }

    if(errorCount>=5) {
      nunchuk.reInit();
      errorCount = 0;
    }

    // Allow other input when Nunchuck does not send speed
    if(motor.pwm != 0.0 && motor.steer != 0.0 ) break; 
  #endif

  #ifdef INPUT_PADDELEC
    paddelec.update(motor.pwm, motor.steer, motor.actualSpeed_kmh, motor.actualSteer_kmh, (uint32_t)deltaMillis);
    if(debug) paddelec.debug(*COM[DEBUG_COM]);

    // Allow other Inputs when Gametraks are not pulled out 
    if(paddelec.gametrak1.r > 500 || paddelec.gametrak2.r > 500) break;
  #endif

  #ifdef INPUT_PADDELECIMU
    paddelec.update(motor.pwm, motor.steer, motor.actualSpeed_kmh, motor.actualSteer_kmh, (uint32_t)deltaMillis);
    if(debug) paddelec.debug(*COM[DEBUG_COM]);
  #endif
  
  } while(false);
  
#ifdef OLED
    uint mX = u8g2.getDisplayWidth()/2; 
    uint mY = u8g2.getDisplayHeight()/2;

    uint motorX = mX+(motor.steer/1000.0*(double)mY);
    uint motorY = mY-(motor.pwm/1000.0*(double)mY);
    
  #ifdef INPUT_IMU
    double aX =  imu.ax / 32768.0 * u8g2.getDisplayHeight()/2.0;
    double aY = -imu.ay / 32768.0 * u8g2.getDisplayWidth() /2.0;
    double aZ =  imu.az / 32768.0 * u8g2.getDisplayHeight()/2.0;

    double gX =  imu.gx / 32768.0 * u8g2.getDisplayWidth() /2.0;
    double gY =  imu.gy / 32768.0 * u8g2.getDisplayHeight()/2.0;
    double gZ =  imu.gz / 32768.0 * u8g2.getDisplayHeight()/2.0;
  #endif

  #ifdef INPUT_PADDELECIMU
    double pwmR=0.0, pwmL=0.0;
    paddelec.steerToRL(motor.steer, motor.pwm, pwmR, pwmL);
    pwmR = -pwmR / 1000.0 * u8g2.getDisplayHeight()/2.0;
    pwmL = -pwmL / 1000.0 * u8g2.getDisplayHeight()/2.0;

    double pitchangle = paddelec.imu.pitchangle();

  #endif

  u8g2.firstPage();  
  
  do {
    u8g2_prepare();

  #ifdef INPUT_IMU
    if(aX>0) u8g2.drawFrame(0    ,mY   ,1, aX);
    else     u8g2.drawFrame(0    ,mY+aX,1,-aX);

    if(aY>0) u8g2.drawFrame(   mX,0, aY,1);
    else     u8g2.drawFrame(mX+aY,0,-aY,1);
    
    if(aZ>0) u8g2.drawFrame(u8g2.getDisplayWidth()-1    ,mY   ,1, aZ);
    else     u8g2.drawFrame(u8g2.getDisplayWidth()-1    ,mY+aZ,1,-aZ);

    if(gY>0) u8g2.drawFrame(2    ,mY   ,1, gY);
    else     u8g2.drawFrame(2    ,mY+gY,1,-gY);

    if(gX>0) u8g2.drawFrame(   mX,2, gX,1);
    else     u8g2.drawFrame(mX+gX,2,-gX,1);
    
    if(gZ>0) u8g2.drawFrame(u8g2.getDisplayWidth()-3    ,mY   ,1, gZ);
    else     u8g2.drawFrame(u8g2.getDisplayWidth()-3    ,mY+gZ,1,-gZ);
  #endif
  #ifdef INPUT_PADDELECIMU

    if(pwmL>0) u8g2.drawFrame(0    ,mY   ,1, pwmL);
    else     u8g2.drawFrame(0    ,mY+pwmL,1,-pwmL);


    if(pwmR>0) u8g2.drawFrame(u8g2.getDisplayWidth()-1    ,mY   ,1, pwmR);
    else     u8g2.drawFrame(u8g2.getDisplayWidth()-1    ,mY+pwmR,1,-pwmR);
    u8g2.setCursor(0,mY+30);
    u8g2.printf("Pitch%4.0f", pitchangle);

  #endif

    u8g2.setCursor(5,5);
    u8g2.printf("%4.0f %4.0f", motor.pwm, motor.steer);

    u8g2.drawLine(mX,mY,motorX,motorY);

  #ifdef INPUT_ESPnowSLAVE
    u8g2.setCursor(5,15);
    u8g2.printf("%4i", ESPnowdata);
  #endif
  } while( u8g2.nextPage() );
#endif

#ifdef MULTITASKING
    vTaskDelay(20);
  }
}


void motorCommunication( void * pvparameters) {
  int taskno = (int)pvparameters;
  while(1) {
#endif //MULTITASKING

#ifdef OUTPUT_ESPnowMASTER
    if(!isPaired) {
      ScanForSlave();
      // If Slave is found, it would be populated in `slave` variable
      // We will check if `slave` is defined and then we proceed further
      if (slave.channel == CHANNEL) { // check if slave channel is defined
        // `slave` is defined
        // Add slave as peer if it has not been added already
        isPaired = manageSlave();
      }
    } else {
      sendData((const void *) &motor, sizeof(motor));
    }
#endif
#ifdef OUTPUT_PROTOCOL
    PROTOCOL_MSG newMsg;
    memset((void*)&newMsg,0x00,sizeof(PROTOCOL_MSG));
    PROTOCOL_MSG *msg = &newMsg;
    PROTOCOL_BYTES_WRITEVALS *writevals = (PROTOCOL_BYTES_WRITEVALS *) msg->bytes;
    SPEED_DATA *writespeed = (SPEED_DATA *) writevals->content;

    msg->SOM = PROTOCOL_SOM; //Start of Message;

    writevals->cmd  = PROTOCOL_CMD_WRITEVAL;  // Write value
    writevals->code = 0x03; // speed data from params array  

    writespeed->speed_max_power            =  600;
    writespeed->speed_min_power            = -600;
    writespeed->speed_minimum_speed        =   40;
    writespeed->wanted_speed_mm_per_sec[0] = motor.pwm + motor.steer;
    writespeed->wanted_speed_mm_per_sec[1] = motor.pwm - motor.steer;


    msg->len = sizeof(writevals->cmd) + sizeof(writevals->code) + sizeof(SPEED_DATA) + 1; // 1 for Checksum
    protocol_send(msg);
    delay(MOTORINPUT_PERIOD);      

    while(COM[MOTOR_COM]->available())
        {     
          protocol_byte( COM[MOTOR_COM]->read() );
        }
#endif
#ifdef OUTPUT_BINARY
    /* cast & limit values to a valid range */
    int16_t steer = (int16_t) limit(-1000.0, motor.steer, 1000.0);
    int16_t pwm   = (int16_t) limit(-1000.0, motor.pwm,   1000.0);
    

    /* Send motor pwm values to motor control unit */
    COM[MOTOR_COM]->write((uint8_t *) &steer, sizeof(steer)); 
    COM[MOTOR_COM]->write((uint8_t *) &pwm,   sizeof(pwm));

  #ifdef OUTPUT_BINARY_CRC
    /* calc and send checksum */
    uint32_t crc = 0;
    crc32((const void *)&steer, sizeof(steer), &crc); 
    crc32((const void *)&pwm,   sizeof(pwm),   &crc); 
    
    COM[MOTOR_COM]->write((uint8_t *) &crc,   sizeof(crc));
  #endif

    /* refresh actual motor speed */
    updateSpeed();

    /* debug output */
    #ifdef WIFI
    for(byte cln = 0; cln < MAX_NMEA_CLIENTS; cln++) {   
      if(TCPClient[1][cln]) {                    
        if(debug) TCPClient[1][cln].print(" U: ");
        if(debug) TCPClient[1][cln].printf("%8i %8i\n", pwm, steer);
      } 
    }
    #endif
    if(debug) COM[DEBUG_COM]->printf("\nU: ");
//    if(debug) COM[DEBUG_COM]->printf("%6i %6i %11u ", pwm, steer, crc);
    if(debug) COM[DEBUG_COM]->printf("%6i %6i ", pwm, steer);
    if(debug) COM[DEBUG_COM]->printf("%7.2f %7.2f ", motor.actualSpeed_kmh, motor.actualSteer_kmh);
#endif
#ifdef MULTITASKING
    delay(MOTORINPUT_PERIOD);           
  }
#endif //MULTITASKING
}