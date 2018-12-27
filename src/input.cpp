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

#ifdef INPUT_NUNCHUK
  #include <ArduinoNunchuk.h>
  ArduinoNunchuk nunchuk = ArduinoNunchuk();
  int nunchukReinitCount=0;
  int nunchukTimeout=0;
#endif // INPUT_NUNCHUK

#ifdef INPUT_ESPNOW
  #include "ESP32_espnow_MasterSlave.h"
  volatile int espnowTimeout = 10000;
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

void setupInput() {

    #ifdef INPUT_ESPNOW
    setupEspNow();
    #endif


    #ifdef INPUT_NUNCHUK
    nunchuk.init();
    #endif

    #ifdef INPUT_BLE
    setupBLE();
    #endif

    #ifdef INPUT_IMU
      imu.init();
      #ifdef DEBUG_PLOTTER
//        plot.AddTimeGraph( "IMU acceleration", 1000, "ax", imu.ax, "ay", imu.ay, "az", imu.az);
//        plot.AddTimeGraph( "IMU gyro", 1000, "gx", imu.gx, "gy", imu.gy, "gz", imu.gz);
        plot.AddTimeGraph( "IMU acceleration", 500, "ax", imu.ax);
        plot.AddTimeGraph( "IMU gyro", 500, "gz", imu.gz);
      #endif
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
  #if !defined(INPUT_PADDELEC) && !defined(INPUT_PADDELECIMU) // TODO: Find better way?
    slowReset(motor.setpoint.pwm,   0.0, 10.0);
    slowReset(motor.setpoint.steer, 0.0, 10.0);
  #endif

  #ifdef INPUT_ESPNOW
    // Disable all other Input Methods as soon as data from ESPnow was received
    if(espnowTimeout < 100) {
      espnowTimeout++;
      break;
    }
  #endif
  #ifdef INPUT_BLE
    //  loopBLE();
    motor.setpoint.pwm = ble_pitch;
    motor.setpoint.steer = ble_roll;
    slowReset(ble_pitch, 0.0, 0.1);
    slowReset(ble_roll, 0.0, 0.1);
    break;
  #endif

  #ifdef INPUT_IMU
    //  imu.loopIMU();
    imu.update(motor.setpoint.pwm, motor.setpoint.steer);
    if(debug) imu.debug(*COM[DEBUG_COM]);

    // Allow other Inputs when no Button is pressed
    if(imu.cButton == 1) break;
  #endif

  #ifdef INPUT_PLATOONING
    platooning.update(motor.setpoint.pwm, motor.setpoint.steer);
    if(debug) platooning.debug(*COM[DEBUG_COM]);

    // Allow other Inputs when Gametrak is not pulled out
    if(platooning.gametrak1.getR_mm() > platooning.cfgPlatooning.rActivationThreshold_mm) break;
  #endif

  #if defined(INPUT_NUNCHUK)
    ++nunchukTimeout;
    ++nunchukReinitCount;
    double tempPWM = motor.setpoint.pwm;
    double tempSteer = motor.setpoint.steer;

    if(debug) nunchuk.debug(*COM[DEBUG_COM]);

    switch(nunchuk.update(motor.setpoint.pwm, motor.setpoint.steer)) {
      case NUNCHUK_ERR_COUNT:
        if(debug) COM[DEBUG_COM]->print("Nunchuk: NUNCHUK_ERR_COUNT ");
        break;
      case NUNCHUK_ERR_NOINIT:
        nunchuk.reInit();
        nunchukReinitCount = 0;
        if(debug) COM[DEBUG_COM]->print("Reinit Nunchuk: NUNCHUK_ERR_NOINIT ");
        break;
      case NUNCHUK_ERR_SEND:
        if(debug) COM[DEBUG_COM]->print("Nunchuk: NUNCHUK_ERR_SEND ");
        break;
      case NUNCHUK_ERR_ZERO:
        if(debug) COM[DEBUG_COM]->print("Nunchuk: NUNCHUK_ERR_ZERO ");
        break;
      case NUNCHUK_ERR_DEV3:
        if(debug) COM[DEBUG_COM]->print("Nunchuk: NUNCHUK_ERR_DEV3 ");
        break;
      case NUNCHUK_ERR_DEV4:
        if(debug) COM[DEBUG_COM]->print("Nunchuk: NUNCHUK_ERR_DEV4 ");
        break;
      case NUNCHUK_ERR_DEV5:
        if(debug) COM[DEBUG_COM]->print("Nunchuk: NUNCHUK_ERR_DEV5 ");
        break;
      case NUNCHUK_ERR_DEV6:
        if(debug) COM[DEBUG_COM]->print("Nunchuk: NUNCHUK_ERR_DEV6 ");
        break;
      case NUNCHUK_ERR_DEV7:
        if(debug) COM[DEBUG_COM]->print("Nunchuk: NUNCHUK_ERR_DEV7 ");
        break;
      case NUNCHUK_ERR_DEV1:
        if(debug) COM[DEBUG_COM]->print("Nunchuk: NUNCHUK_ERR_DEV1 (continuing) ");
      case NUNCHUK_ERR_DEV2:
        if(debug) COM[DEBUG_COM]->print("Nunchuk: NUNCHUK_ERR_DEV2 (continuing) ");
      case NUNCHUK_ERR_NOERR:
        nunchukTimeout = 0;
        nunchukReinitCount = 0;
        break;
    }

    // try fixing the Nunchuk by resetting
    if(nunchukReinitCount>=5) {
      nunchuk.reInit();
      if(debug) COM[DEBUG_COM]->print("Reinit Nunchuk ");
      nunchukReinitCount = 0;
    }

    // set safe value when no data is received
    if(nunchukTimeout>=10) {
      motor.setpoint.pwm   = 0.0;
      motor.setpoint.steer = 0.0;
      if(debug) COM[DEBUG_COM]->print("Nunchuk Timeout");
    }

    // Do not allow other inputs to override when Nunchuk has sent data
    if(motor.setpoint.pwm != 0.0 && motor.setpoint.steer != 0.0 ) break;
    else {
      // TODO: dangerous, no defined state when joystick suddenly sends zero
      motor.setpoint.pwm = tempPWM;
      motor.setpoint.steer = tempSteer;
    }
  #endif

  #ifdef INPUT_PADDELEC
    paddelec.update(motor.setpoint.pwm, motor.setpoint.steer, motor.measured.actualSpeed_kmh, motor.measured.actualSteer_kmh, (uint32_t)deltaMillis);
    if(debug) paddelec.debug(*COM[DEBUG_COM]);

    // Allow other Inputs when Gametraks are not pulled out
    if(paddelec.gametrak1.r > 500 || paddelec.gametrak2.r > 500) break;
  #endif

  #ifdef INPUT_PADDELECIMU
    paddelec.update(motor.setpoint.pwm, motor.setpoint.steer, motor.measured.actualSpeed_kmh, motor.measured.actualSteer_kmh, (uint32_t)deltaMillis);
    if(debug) paddelec.debug(*COM[DEBUG_COM]);
  #endif

  } while(false);

  #if defined(DEBUG_PLOTTER) && (defined(INPUT_IMU) || defined(INPUT_PADDELECIMU))
    plot.Plot();
  #endif

#ifdef DEBUG_OLED
    uint mX = u8g2.getDisplayWidth()/2;
    uint mY = u8g2.getDisplayHeight()/2;

    uint motorX = mX+(motor.setpoint.steer/1000.0*(double)mY);
    uint motorY = mY-(motor.setpoint.pwm/1000.0*(double)mY);

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
    paddelec.steerToRL(motor.setpoint.steer, motor.setpoint.pwm, pwmR, pwmL);
    pwmR = -pwmR / 1000.0 * u8g2.getDisplayHeight()/2.0;
    pwmL = -pwmL / 1000.0 * u8g2.getDisplayHeight()/2.0;

    double pitchangle = paddelec.imu.pitchangle() - paddelec.imu.pitch_zero;

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
    u8g2.printf("%4.0f %4.0f", motor.setpoint.pwm, motor.setpoint.steer);

    u8g2.drawLine(mX,mY,motorX,motorY);

  } while( u8g2.nextPage() );
#endif

#ifdef MULTITASKING
    vTaskDelay(10);
  }
#endif //MULTITASKING
}