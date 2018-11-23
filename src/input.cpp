#include "input.h"
#include <Arduino.h>
#include "main.h"
#include "config.h"
#include "serialbridge.h"

#ifdef DEBUG_OLED
  #include "oled.h"
#endif

#if defined(INPUT_PADDELEC) || defined(INPUT_PADDELECIMU)
  #include "Paddelec.h"
  Paddelec paddelec = Paddelec();
#endif // INPUT_PADDELEC

#ifdef INPUT_NUNCHUCK
  #include <ArduinoNunchuk.h>
  ArduinoNunchuk nunchuk = ArduinoNunchuk();
#endif // INPUT_NUNCHUCK

#ifdef INPUT_ESPnowSLAVE
  #include "ESPnowSlave.h"
  bool ESPnowDataReceived = false;
#endif

#ifdef INPUT_PLATOONING
  #include "Platooning.h"
  Platooning platooning = Platooning();
#endif

#ifdef INPUT_BLE
  #include "BLE.h"
#endif

#ifdef INPUT_IMU
  #include <IMU.h>
  Imu imu = Imu();
#endif

uint32_t millisMotorcomm = 0;      // virtual timer for motor update
int errorCount=0;

void setupInput() {
    
    #ifdef INPUT_ESPnowSLAVE
    setupESPnowSlave();
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
}


// Incrementally decrease variable
void slowReset(double &variable, double goal, double step) {
  if      ((variable - goal) > step) variable -= step;
  else if ((goal - variable) > step) variable += step;
  else                               variable  = goal;
}


void mainloop( void *pvparameters ) {
#ifdef MULTITASKING
  int taskno = (int)pvparameters;
  while(1) {
#endif //MULTITASKING

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
  
#ifdef DEBUG_OLED
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
#endif //MULTITASKING
}