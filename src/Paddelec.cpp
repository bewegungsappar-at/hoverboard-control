
#include "Paddelec.h"
#include "serialbridge.h"
#include "main.h"

#if defined(INPUT_PADDELECIMU)

#define INPUT_PADDELEC_DEBUG false


void Paddelec::update(volatile double &pwm, volatile  double &steer, volatile  double &actualSpeed_kmh, volatile  double &actualSteer_kmh, volatile  uint32_t deltaMillis) {
  double pwmR      =0, pwmL      =0;
  double speedR_kmh=0, speedL_kmh=0;
  imu.update();

  /* convert from speed and steering to left and right wheel speed */
  steerToRL(steer,           pwm,             pwmR,       pwmL);
  steerToRL(actualSteer_kmh, actualSpeed_kmh, speedR_kmh, speedL_kmh);


  #define PADDELEC_INERTIA_THRESHOLD  10.0
  #define PADDELEC_INERTIA_OFFSET    150.0

  /* Remove Offset added to overcome intial inertia */
  if(pwmL >  PADDELEC_INERTIA_OFFSET) pwmL -= PADDELEC_INERTIA_OFFSET;
  if(pwmR >  PADDELEC_INERTIA_OFFSET) pwmR -= PADDELEC_INERTIA_OFFSET;
  if(pwmL < -PADDELEC_INERTIA_OFFSET) pwmL += PADDELEC_INERTIA_OFFSET;
  if(pwmR < -PADDELEC_INERTIA_OFFSET) pwmR += PADDELEC_INERTIA_OFFSET;


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

  if(INPUT_PADDELEC_DEBUG) COM[DEBUG_COM]->print("PD: ");

  /* process paddle strokes */
  if       (paddleAngle > cfgPaddle.paddleAngleThreshold) {    // gametrak2 side of paddle is down

    /* get speed difference between paddle and "water". Paddling slower than current speed should slow down. */
    double speedDelta = (-imu.gz * cfgPaddle.deltaRtoSpeed) - (speedL_kmh * 10);

    /* update speed and apply crosstalk */
    pwmL += ( speedDelta * cfgPaddle.pwmMultiplier * deltaMillis );
    pwmR += ( speedDelta * cfgPaddle.pwmMultiplier * deltaMillis * cfgPaddle.crosstalkLR );
    if(INPUT_PADDELEC_DEBUG) COM[DEBUG_COM]->print("g2L ");
    if(INPUT_PADDELEC_DEBUG) COM[DEBUG_COM]->printf("%6i %6i %6i ",(int)(imu.gz * cfgPaddle.deltaRtoSpeed), (int)speedL_kmh, (int)speedDelta);


  } else if(paddleAngle <  -cfgPaddle.paddleAngleThreshold) {    // gametrak1 side of paddle is down

    /* get speed difference between paddle and "water". Paddling slower than current speed should slow down. */
    double speedDelta = (imu.gz * cfgPaddle.deltaRtoSpeed) - (speedR_kmh * 10);

    /* update speed and apply crosstalk */
    pwmR += ( speedDelta * cfgPaddle.pwmMultiplier * deltaMillis );
    pwmL += ( speedDelta * cfgPaddle.pwmMultiplier * deltaMillis * cfgPaddle.crosstalkLR );
    if(INPUT_PADDELEC_DEBUG) COM[DEBUG_COM]->print("g1R ");
    if(INPUT_PADDELEC_DEBUG) COM[DEBUG_COM]->printf("%6i %6i %6i ",(int)(-imu.gz * cfgPaddle.deltaRtoSpeed), (int)speedR_kmh, (int)speedDelta);
  } else  {
    if(INPUT_PADDELEC_DEBUG) COM[DEBUG_COM]->print("___ ");
    if(INPUT_PADDELEC_DEBUG) COM[DEBUG_COM]->printf("%6i %6i %6i ", 0, 0, 0);
  }

  if(imu.az > 80) {
    slowReset(steer, 0, 300);
    slowReset(pwm, 0, 300);
    return;
  }

  /* Add Offset to overcome intial inertia */
  if(pwmL >  PADDELEC_INERTIA_THRESHOLD) pwmL += PADDELEC_INERTIA_OFFSET;
  if(pwmR >  PADDELEC_INERTIA_THRESHOLD) pwmR += PADDELEC_INERTIA_OFFSET;
  if(pwmL < -PADDELEC_INERTIA_THRESHOLD) pwmL -= PADDELEC_INERTIA_OFFSET;
  if(pwmR < -PADDELEC_INERTIA_THRESHOLD) pwmR -= PADDELEC_INERTIA_OFFSET;






/* Limit Maximum Output */
  #define PADDELEC_LIMIT 1500

  if(pwmL >  PADDELEC_LIMIT) pwmL =  PADDELEC_LIMIT;
  if(pwmL < -PADDELEC_LIMIT) pwmL = -PADDELEC_LIMIT;
  if(pwmR >  PADDELEC_LIMIT) pwmR =  PADDELEC_LIMIT;
  if(pwmR < -PADDELEC_LIMIT) pwmR = -PADDELEC_LIMIT;



  /* convert from left and right wheel speed to speed and steering */
  RLpwmToSteer(steer, pwm, pwmR, pwmL);
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
  steer = pwmL - pwm;
}

void Paddelec::steerToRL(volatile double &steer, volatile double &pwm, double &pwmR, double &pwmL)
{
  pwmR = pwm - steer;
  pwmL = pwm + steer;
}

// Incrementally decrease variable
void Paddelec::slowReset(volatile double &variable, double goal, double step) {
  if      ((variable - goal) > step) variable -= step;
  else if ((goal - variable) > step) variable += step;
  else                               variable  = goal;
}

#endif
