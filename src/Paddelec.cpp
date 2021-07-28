
#include "Paddelec.h"
#include "serialbridge.h"
#include "main.h"
#include "config.h"


    bool Paddelec::init()
    {
      Wire.begin();
    #ifdef INPUT_IMU_BNO0805
     if (imu.begin(BNO080_DEFAULT_ADDRESS, Wire,INT_PIN) == false)
      {
        Serial.print("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
        while (1);
      }

      Wire.setClock(400000); //Increase I2C data rate to 400kHz

      imu.enableRotationVector(50); //Send data update every 50ms
     // if(debug) COM[DEBUG_COM]->print(F("Rotation vector enabled"));
     // if(debug) COM[DEBUG_COM]->print(F("Output in form i, j, k, real, accuracy"));
      imu.enableActivityClassifier(50, enableActivities, activityConfidences);

      // Joystik
      pinMode(JOYPIN_Z, INPUT);
    #else
      imu.init();
    #endif
    /*
      cfgPaddle.paddleAngleThreshold =   22.0;      // activation angle threshold of paddle. Below threshold, paddle is not enganged and paddelec is freewheeling.
      cfgPaddle.deltaRtoSpeed        =    0.00025;   // (spassfaktor) conversion factor between paddle movement to speed. This defines also the maximum speed.
      cfgPaddle.pwmMultiplier        =    0.10;      // (spassfaktor) effect of paddle stroke to speed
      cfgPaddle.crosstalkLR          =    0.0;      // multiplier for steering
      cfgPaddle.realign              =    0.0;      // paddelc tries to go straight forward
      cfgPaddle.drag                 =    0.0002;   // drag/water resistance
      cfgPaddle.flipControl          =    1;        // 1: Normal. -1 Flipped
      cfgPaddle.maxValidSpeed        =   15.0;      // All speed inputs above this threshold are considered unplausible and therefore faulty
      cfgPaddle.maxValidSteer        =   15.0;      // All steer inputs above this threshold are considered unplausible and therefore faulty
      cfgPaddle.maxValidGyro         =   (int16_t)(40.0 / cfgPaddle.deltaRtoSpeed);      // All steer inputs above this threshold are considered unplausible and therefore faulty
      cfgPaddle.inertiaThreshold     =   20.0;      // Minimum value to get vehicle moving
      cfgPaddle.inertiaOffset        =   50.0;      // Offset to set vehicle in motion
      cfgPaddle.debugMode            = true;        // enable debug output
      cfgPaddle.pwmLimit             = 1500.0;      // Limit maximum PWM to this value

*/

      cfgPaddle.paddleAngleThreshold =   0.4;      // activation angle threshold of paddle. Below threshold, paddle is not enganged and paddelec is freewheeling.
      cfgPaddle.deltaRtoSpeed        =    20;   // (spassfaktor) conversion factor between paddle movement to speed. This defines also the maximum speed.
      cfgPaddle.pwmMultiplier        =    0.10;      // (spassfaktor) effect of paddle stroke to speed
      cfgPaddle.crosstalkLR          =    0.0;      // multiplier for steering
      cfgPaddle.realign              =    0.0;      // paddelc tries to go straight forward
      cfgPaddle.drag                 =    0.0002;   // drag/water resistance
      cfgPaddle.flipControl          =    1;        // 1: Normal. -1 Flipped
      cfgPaddle.maxValidSpeed        =   15.0;      // All speed inputs above this threshold are considered unplausible and therefore faulty
      cfgPaddle.maxValidSteer        =   15.0;      // All steer inputs above this threshold are considered unplausible and therefore faulty
      cfgPaddle.maxValidGyro         =   (int16_t)(40.0 / cfgPaddle.deltaRtoSpeed);      // All steer inputs above this threshold are considered unplausible and therefore faulty
      cfgPaddle.inertiaThreshold     =   20.0;      // Minimum value to get vehicle moving
      cfgPaddle.inertiaOffset        =   50.0;      // Offset to set vehicle in motion
      cfgPaddle.debugMode            = true;        // enable debug output
      cfgPaddle.pwmLimit             = 1500.0;      // Limit maximum PWM to this value

# ifdef PADDELEC_STOPSWITCH_PIN1
    #if (CONFIGURATION_SET == CFG_PAGAIE)
      pinMode(PADDELEC_STOPSWITCH_PIN1, INPUT_PULLUP);
    #else
      pinMode(PADDELEC_STOPSWITCH_PIN1, INPUT_PULLDOWN);
    #endif
# endif


# ifdef PAGAIE_STOPSWITCH_PIN1
 # endif

# ifdef PADDELEC_STOPSWITCH_PIN2
      pinMode(PADDELEC_STOPSWITCH_PIN2, OUTPUT);
      digitalWrite(PADDELEC_STOPSWITCH_PIN2, HIGH);
# endif

#ifdef INPUT_IMU_BNO0805

#else
      delay(1000);                  // Wait till shaking from switching on is gone
      imu.pitch_zero = imu.pitchangle();
#endif


      return(true);
    }



void Paddelec::update(volatile double &pwm, volatile  double &steer, volatile  double &actualSpeed_kmh, volatile  double &actualSteer_kmh, volatile  uint32_t deltaMillis)
{
  #ifdef INPUT_IMU_BNO0805

  int px=0;
  int py=0;

  /* Joistik or Gyro Switch */
  if( digitalRead(JOYPIN_Z))
  {
    joystik_mode_change_request = true;
  }
  else
  {
    if(joystik_mode_change_request)
    {
      joystik_mode = !joystik_mode;
      joystik_mode_change_request=false;
    }
  }

/* Joystik */
  if(joystik_mode)
  {
    py=analogRead(JOYPIN_X);
    px=analogRead(JOYPIN_Y);

    //COM[DEBUG_COM]->printf(" pagaie: %d - %d  ..  ",px,py);
    //Serial.printf("pagaie: %d - %d  ..  ",px,py);

    px = map(px,700,3200,-300,300);
    py = map(py,700,3200,-300,300);

    //COM[DEBUG_COM]->printf(" gemappt: %d - %d  ..  ", px,py);
    //Serial.printf("gemappt: %d - %d  \n", px,py);

    if(!( px+50 < 100 && px-50 > -100 && py+50 < 100 && py-50 > -100))
    {
      if(px>0){ px=px-100; } else {px=px+100;}
      if(py>0){ py=py-100;} else {py=py+100;}
      steer=px;
      pwm=py;

      //COM[DEBUG_COM]->printf(" speed: %d - %d  .. \n",px,py);
    }
    else
    {
      slowReset(steer, 0, 100, 0);
      slowReset(pwm, 0, 100, 0);
      //COM[DEBUG_COM]->printf(" speed null \n");
    }
    bno0805_last_yaw = bno0805_yaw;

  }
  else  /* GYRO BNO0805 */
  {

  double pwmR      =0, pwmL      =0;
  double speedR_kmh=0, speedL_kmh=0;
    double speedDelta=0;
    double yawDelta =0;

    /* Check if speed and Steer are in a valid range */
    if( actualSpeed_kmh < -cfgPaddle.maxValidSpeed || actualSpeed_kmh > cfgPaddle.maxValidSpeed)
    {
      if(cfgPaddle.debugMode) COM[DEBUG_COM]->printf("\nExceeded Speed limit. Speed %5.1f Steer %5.1f\n", actualSpeed_kmh, actualSteer_kmh);
      return;  // Abort processing.
    }

    if( actualSteer_kmh < -cfgPaddle.maxValidSteer || actualSteer_kmh > cfgPaddle.maxValidSteer)
    {
      if(cfgPaddle.debugMode) COM[DEBUG_COM]->printf("\nExceeded Steer limit. Speed %5.1f Steer %5.1f\n", actualSpeed_kmh, actualSteer_kmh);
      return;  // Abort processing.
    }

    /* Check if Gyro Input is in a valid range */
    if( yawDelta < -cfgPaddle.maxValidGyro || yawDelta > cfgPaddle.maxValidGyro)
    {
      if(cfgPaddle.debugMode) COM[DEBUG_COM]->printf("\nExceeded Gyro Speed. Paddle Speed %5.1f GYRO %5f\n", yawDelta * cfgPaddle.deltaRtoSpeed, yawDelta);
      return;  // Abort processing.
    }

    /* convert from speed and steering to left and right wheel speed */
    steerToRL(steer,           pwm,             pwmL,       pwmR);
    steerToRL(actualSteer_kmh, actualSpeed_kmh, speedR_kmh, speedL_kmh);


    /* Remove Offset added to overcome intial inertia */
    if(pwmL >  cfgPaddle.inertiaOffset) pwmL -= cfgPaddle.inertiaOffset;
    if(pwmR >  cfgPaddle.inertiaOffset) pwmR -= cfgPaddle.inertiaOffset;
    if(pwmL < -cfgPaddle.inertiaOffset) pwmL += cfgPaddle.inertiaOffset;
    if(pwmR < -cfgPaddle.inertiaOffset) pwmR += cfgPaddle.inertiaOffset;




    /* get values from BNO0805 */
    if (imu.dataAvailable() == true)
    {
      bno0805_yaw = imu.getYaw();
      bno0805_roll = imu.getRoll();


      yawDelta = bno0805_yaw-bno0805_last_yaw;
      //Serial.printf("\n Gyro  %5.1f GYRO %5f ..\n", yawDelta, bno0805_roll);


      /*Left or Right paddle down ? */
      double paddleAngle = cfgPaddle.flipControl * (bno0805_roll-pitchangle_zero);
      //Serial.printf("\n paddleAngle  %5.2f - bno  %5.2f - pitchangle  %5.2f ..\n", paddleAngle, bno0805_roll,pitchangle_zero);


      /*process paddle strokes */
      if(paddleAngle > cfgPaddle.paddleAngleThreshold)     //  LEFT side of paddle is down
      {
        /* get speed difference between paddle and "water". Paddling slower than current speed should slow down. */
        speedDelta = (-yawDelta * cfgPaddle.deltaRtoSpeed) - speedL_kmh;
        /* update speed and apply crosstalk */
        pwmL += ( speedDelta * cfgPaddle.pwmMultiplier * deltaMillis * cfgPaddle.crosstalkLR);
        pwmR += ( speedDelta * cfgPaddle.pwmMultiplier * deltaMillis  );
        //if(cfgPaddle.debugMode) COM[DEBUG_COM]->print("g2L ");
        //if(cfgPaddle.debugMode) COM[DEBUG_COM]->printf("%5.2f %5.2f %5.2f ",yawDelta * cfgPaddle.deltaRtoSpeed, speedL_kmh, speedDelta);
        Serial.printf(" LEFT yawDelta %5.2f pwmL  %5.2f pwmR  %5.2f speedDelta %5f \n", -yawDelta, pwmL, pwmR, speedDelta);
      }
      else if(paddleAngle <  -cfgPaddle.paddleAngleThreshold)  // RIGHT side of paddle is down
      {
        /* get speed difference between paddle and "water". Paddling slower than current speed should slow down. */
        speedDelta = (yawDelta * cfgPaddle.deltaRtoSpeed) - speedR_kmh;
        /* update speed and apply crosstalk */
        pwmR += ( speedDelta * cfgPaddle.pwmMultiplier * deltaMillis * cfgPaddle.crosstalkLR);
        pwmL += ( speedDelta * cfgPaddle.pwmMultiplier * deltaMillis  );
        //if(cfgPaddle.debugMode) COM[DEBUG_COM]->print("g1R ");
        //if(cfgPaddle.debugMode) COM[DEBUG_COM]->printf("%5.2f %5.2f %5.2f ",-yawDelta * cfgPaddle.deltaRtoSpeed, speedR_kmh, speedDelta);
        Serial.printf(" RIGHT yawDelta %5.2f pwmL  %5.2f pwmR  %5.2f speedDelta %5f \n", yawDelta, pwmL, pwmR, speedDelta);
      }
      else
      {
        //if(cfgPaddle.debugMode) COM[DEBUG_COM]->print("___ ");
        //if(cfgPaddle.debugMode) COM[DEBUG_COM]->printf("%5.2f %5.2f %5.2f ", 0.0, 0.0, 0.0);
        Serial.printf(" IDLE bno0805_yaw %5.2f pwmL  %5.2f pwmR  %5.2f speedDelta %5f \n", -bno0805_yaw, pwmL, pwmR, speedDelta);

        /* paddle out of water .. ignore position change */
      }
    }


    bno0805_last_yaw = bno0805_yaw;

    /* simulate drag */
    /* decrease pwm by a factor each time function is called */
    pwmL *= 1.0 - (cfgPaddle.drag * deltaMillis);
    pwmR *= 1.0 - (cfgPaddle.drag * deltaMillis);

    /* Kajak tries to align itself straight */
    pwmL += (pwmR - pwmL) * (cfgPaddle.realign * deltaMillis);
    pwmR += (pwmL - pwmR) * (cfgPaddle.realign * deltaMillis);


    /* Add Offset to overcome intial inertia */
    if(pwmL >  cfgPaddle.inertiaThreshold) pwmL += cfgPaddle.inertiaOffset;
    if(pwmR >  cfgPaddle.inertiaThreshold) pwmR += cfgPaddle.inertiaOffset;
    if(pwmL < -cfgPaddle.inertiaThreshold) pwmL -= cfgPaddle.inertiaOffset;
    if(pwmR < -cfgPaddle.inertiaThreshold) pwmR -= cfgPaddle.inertiaOffset;




  /* Limit Maximum Output */
    if(pwmL >  cfgPaddle.pwmLimit) pwmL =  cfgPaddle.pwmLimit;
    if(pwmL < -cfgPaddle.pwmLimit) pwmL = -cfgPaddle.pwmLimit;
    if(pwmR >  cfgPaddle.pwmLimit) pwmR =  cfgPaddle.pwmLimit;
    if(pwmR < -cfgPaddle.pwmLimit) pwmR = -cfgPaddle.pwmLimit;


    //Serial.printf("\n .. steer %5.2f pwm %5.2f pwmL %5.2f pwmR %5.2f\n", steer, pwm,pwmL,pwmR);

    /* convert from left and right wheel speed to speed and steering */
    RLpwmToSteer(steer, pwm, pwmL, pwmR);
  }
}

void Paddelec::debug(Stream &port)
{
 // imu.debug(port);//
 // port.printf("P  %6.2f ", bno0805_yaw);
}

void Paddelec::RLpwmToSteer(volatile double &steer, volatile double &pwm, double &pwmR, double &pwmL)
{
  pwm = (pwmR + pwmL) / 2;
  steer = pwmR - pwm;
}

void Paddelec::steerToRL(volatile double &steer, volatile double &pwm, double &pwmR, double &pwmL)
{
  pwmR = pwm + steer;
  pwmL = pwm - steer;
}

// Incrementally decrease variable
void Paddelec::slowReset(volatile double &variable, double goal, double step, double fact)
{
  variable  += (goal - variable) * fact;
  if      ((variable - goal) > step) variable -= step;
  else if ((goal - variable) > step) variable += step;
  else                               variable  = goal;
}


#else

  double pwmR      =0, pwmL      =0;
  double speedR_kmh=0, speedL_kmh=0;
  imu.update();
  int mode_switch = 0;
  int pagaie_x0 = 0;
  int pagaie_y0 = 0;

  /* Check if speed and Steer are in a valid range */
  if( actualSpeed_kmh < -cfgPaddle.maxValidSpeed || actualSpeed_kmh > cfgPaddle.maxValidSpeed)
  {
    if(cfgPaddle.debugMode) COM[DEBUG_COM]->printf("\nExceeded Speed limit. Speed %5.1f Steer %5.1f\n", actualSpeed_kmh, actualSteer_kmh);
    return;  // Abort processing.
  }

  if( actualSteer_kmh < -cfgPaddle.maxValidSteer || actualSteer_kmh > cfgPaddle.maxValidSteer)
  {
    if(cfgPaddle.debugMode) COM[DEBUG_COM]->printf("\nExceeded Steer limit. Speed %5.1f Steer %5.1f\n", actualSpeed_kmh, actualSteer_kmh);
    return;  // Abort processing.
  }


  /* Check if Gyro Input is in a valid range */
  if( imu.gz < -cfgPaddle.maxValidGyro || imu.gz > cfgPaddle.maxValidGyro)
  {
    if(cfgPaddle.debugMode) COM[DEBUG_COM]->printf("\nExceeded Gyro Speed. Paddle Speed %5.1f GYRO %5i\n", imu.gz * cfgPaddle.deltaRtoSpeed, imu.gz);
    return;  // Abort processing.
  }


  /* convert from speed and steering to left and right wheel speed */
  steerToRL(steer,           pwm,             pwmL,       pwmR);
  steerToRL(actualSteer_kmh, actualSpeed_kmh, speedR_kmh, speedL_kmh);


  /* Remove Offset added to overcome intial inertia */
  if(pwmL >  cfgPaddle.inertiaOffset) pwmL -= cfgPaddle.inertiaOffset;
  if(pwmR >  cfgPaddle.inertiaOffset) pwmR -= cfgPaddle.inertiaOffset;
  if(pwmL < -cfgPaddle.inertiaOffset) pwmL += cfgPaddle.inertiaOffset;
  if(pwmR < -cfgPaddle.inertiaOffset) pwmR += cfgPaddle.inertiaOffset;


  /* simulate drag */
  /* decrease pwm by a factor each time function is called */
  pwmL *= 1.0 - (cfgPaddle.drag * deltaMillis);
  pwmR *= 1.0 - (cfgPaddle.drag * deltaMillis);

  /* Kajak tries to align itself straight */
  pwmL += (pwmR - pwmL) * (cfgPaddle.realign * deltaMillis);
  pwmR += (pwmL - pwmR) * (cfgPaddle.realign * deltaMillis);


  double paddleAngle = cfgPaddle.flipControl * (imu.pitchangle() - imu.pitch_zero);

  if(cfgPaddle.debugMode) COM[DEBUG_COM]->print("PD: ");

  /* process paddle strokes */
  if       (paddleAngle > cfgPaddle.paddleAngleThreshold)     //  side of paddle is down
  {
    /* get speed difference between paddle and "water". Paddling slower than current speed should slow down. */
    double speedDelta = (-imu.gz * cfgPaddle.deltaRtoSpeed) - speedL_kmh;

    /* update speed and apply crosstalk */
    pwmL += ( speedDelta * cfgPaddle.pwmMultiplier * deltaMillis );
    pwmR += ( speedDelta * cfgPaddle.pwmMultiplier * deltaMillis * cfgPaddle.crosstalkLR );
    if(cfgPaddle.debugMode) COM[DEBUG_COM]->print("g2L ");
    if(cfgPaddle.debugMode) COM[DEBUG_COM]->printf("%5.2f %5.2f %5.2f ",imu.gz * cfgPaddle.deltaRtoSpeed, speedL_kmh, speedDelta);
  }
  else if(paddleAngle <  -cfgPaddle.paddleAngleThreshold)  // gametrak1 side of paddle is down
  {

    /* get speed difference between paddle and "water". Paddling slower than current speed should slow down. */
    double speedDelta = (imu.gz * cfgPaddle.deltaRtoSpeed) - speedR_kmh;

    /* update speed and apply crosstalk */
    pwmR += ( speedDelta * cfgPaddle.pwmMultiplier * deltaMillis );
    pwmL += ( speedDelta * cfgPaddle.pwmMultiplier * deltaMillis * cfgPaddle.crosstalkLR );
    if(cfgPaddle.debugMode) COM[DEBUG_COM]->print("g1R ");
    if(cfgPaddle.debugMode) COM[DEBUG_COM]->printf("%5.2f %5.2f %5.2f ",-imu.gz * cfgPaddle.deltaRtoSpeed, speedR_kmh, speedDelta);
  }
  else
  {
    if(cfgPaddle.debugMode) COM[DEBUG_COM]->print("___ ");
    if(cfgPaddle.debugMode) COM[DEBUG_COM]->printf("%5.2f %5.2f %5.2f ", 0.0, 0.0, 0.0);
  }

  /* Panic Stop */
#ifdef PADDELEC_STOPSWITCH_PIN1
  if( !digitalRead(PADDELEC_STOPSWITCH_PIN1) )
#else
  if(imu.az > 80)
#endif
  {
    #if (CONFIGURATION_SET == CFG_PAGAIE)
    int px=analogRead(A6);
    int py=analogRead(A7);
    COM[DEBUG_COM]->printf(" pagaie: %d - %d  ..  ",px,py);

    px = map(px,700,3200,-600,600);
    py = map(py,700,3200,-600,600);


    COM[DEBUG_COM]->printf(" gemappt: %d - %d  ..  ", px,py);

    if(!( px+50 < 100 && px-50 > -100 && py+50 < 100 && py-50 > -100))
    {
      if(px>0){ px=px-100; } else {px=px+100;}
      if(py>0){ py=py-100;} else {py=py+100;}
      steer=px;
      pwm=py;

      COM[DEBUG_COM]->printf(" speed: %d - %d  .. ",px,py);
    }
    else
  {
    slowReset(steer, 0, 100, 0);
    slowReset(pwm, 0, 100, 0);
            COM[DEBUG_COM]->printf(" speed null ");
    }


/*
        if(mode_switch==0)
        {
          mode_switch=1;
          pagaie_x0 = px;
          pagaie_y0 = py;
          COM[DEBUG_COM]->printf(" modeswitch=0 ");
        }
        else
        {
          if( px+50 < pagaie_x0 && px-100 > pagaie_x0 && py+100 < pagaie_y0 && py-50 > pagaie_y0)
          {
            slowReset(steer, 0, 100, 0);
            slowReset(pwm, 0, 100, 0);
            COM[DEBUG_COM]->printf(" speed null ");

          }
          else
          {
             if(px>0){ px=px-100; } else {px=px+100;}
             if(py>0){ py=py-100;} else {py=py+100;}
             steer=px;
             pwm=py;

             COM[DEBUG_COM]->printf(" speed: %d - %d  .. ",px,py);
          }
        } */
    #else
        slowReset(steer, 0, 100, 0);
        slowReset(pwm, 0, 100, 0);
    #endif
    return;
  }

  /* Add Offset to overcome intial inertia */
  if(pwmL >  cfgPaddle.inertiaThreshold) pwmL += cfgPaddle.inertiaOffset;
  if(pwmR >  cfgPaddle.inertiaThreshold) pwmR += cfgPaddle.inertiaOffset;
  if(pwmL < -cfgPaddle.inertiaThreshold) pwmL -= cfgPaddle.inertiaOffset;
  if(pwmR < -cfgPaddle.inertiaThreshold) pwmR -= cfgPaddle.inertiaOffset;




/* Limit Maximum Output */
  if(pwmL >  cfgPaddle.pwmLimit) pwmL =  cfgPaddle.pwmLimit;
  if(pwmL < -cfgPaddle.pwmLimit) pwmL = -cfgPaddle.pwmLimit;
  if(pwmR >  cfgPaddle.pwmLimit) pwmR =  cfgPaddle.pwmLimit;
  if(pwmR < -cfgPaddle.pwmLimit) pwmR = -cfgPaddle.pwmLimit;



  /* convert from left and right wheel speed to speed and steering */
  RLpwmToSteer(steer, pwm, pwmL, pwmR);
}

void Paddelec::debug(Stream &port)
{
  imu.debug(port);//
}

void Paddelec::RLpwmToSteer(volatile double &steer, volatile double &pwm, double &pwmR, double &pwmL)
{
  pwm = (pwmR + pwmL) / 2;
  steer = pwmR - pwm;
}

void Paddelec::steerToRL(volatile double &steer, volatile double &pwm, double &pwmR, double &pwmL)
{
  pwmR = pwm + steer;
  pwmL = pwm - steer;
}

// Incrementally decrease variable
void Paddelec::slowReset(volatile double &variable, double goal, double step, double fact)
{
  variable  += (goal - variable) * fact;
  if      ((variable - goal) > step) variable -= step;
  else if ((goal - variable) > step) variable += step;
  else                               variable  = goal;
}

 #endif