
#include "Paddelec.h"
#include "serialbridge.h"
#include "main.h"
#include "config.h"


void Paddelec::update(volatile double &pwm, volatile  double &steer, volatile  double &actualSpeed_kmh, volatile  double &actualSteer_kmh, volatile  uint32_t deltaMillis)
{
  double pwmR      =0, pwmL      =0;
  double speedR_kmh=0, speedL_kmh=0;
  imu.update();



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
    slowReset(steer, 0, 100, 0);
    slowReset(pwm, 0, 100, 0);
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