
#include "Paddelec.h"
#include "serialbridge.h"
#include "main.h"
#include "config.h"


void Paddelec::update(volatile double &pwm, volatile  double &steer, volatile  double &actualSpeed_kmh, volatile  double &actualSteer_kmh, volatile  uint32_t deltaMillis)
{
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
//  plotterTempDouble[2] = paddleAngle;
//  plotterTempDouble[3] = imu.az;

  if(cfgPaddle.debugMode) COM[DEBUG_COM]->print("PD: ");

  /* process paddle strokes */
  if       (paddleAngle > cfgPaddle.paddleAngleThreshold)     // gametrak2 side of paddle is down
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
//  plotterTempDouble[0] = imu.ax;
//  plotterTempDouble[1] = imu.ay;


//  plotterTempDouble[1] = imu.gz *0.1;
}

void Paddelec::debug(Stream &port)
{
  imu.debug(port);//
//  port.printf("P %5i %5i %6.2f ", imu.gz, -imu.gz, gametrak1.getTheta_deg() - gametrak2.getTheta_deg());
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