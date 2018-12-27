
#include "Paddelec.h"
#include "serialbridge.h"
#include "main.h"

#if defined(INPUT_PADDELECIMU) || defined(INPUT_PADDELEC)

#define INPUT_PADDELEC_DEBUG false


void Paddelec::update(double &pwm, double &steer, double &actualSpeed_kmh, double &actualSteer_kmh, uint32_t deltaMillis) {
  double pwmR      =0, pwmL      =0;
  double speedR_kmh=0, speedL_kmh=0;
#ifdef INPUT_PADDELEC
  gametrak1.update();
  gametrak2.update();
#elif INPUT_PADDELECIMU
  imu.update();
#endif

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


#ifdef INPUT_PADDELEC
  double paddleAngle = gametrak1.getTheta_deg() - gametrak2.getTheta_deg();
#elif INPUT_PADDELECIMU
  double paddleAngle = cfgPaddle.flipControl * (imu.pitchangle() - imu.pitch_zero);
#endif
//  plotterTempDouble[2] = paddleAngle;
//  plotterTempDouble[3] = imu.az;

  if(INPUT_PADDELEC_DEBUG) COM[DEBUG_COM]->print("PD: ");

  /* process paddle strokes */
  if       (paddleAngle > cfgPaddle.paddleAngleThreshold) {    // gametrak2 side of paddle is down

    /* get speed difference between paddle and "water". Paddling slower than current speed should slow down. */
#ifdef INPUT_PADDELEC
    double speedDelta = ((gametrak2.r - gametrak2.r_last) * cfgPaddle.deltaRtoSpeed) - speedL_kmh;
#elif INPUT_PADDELECIMU
    double speedDelta = (-imu.gz * cfgPaddle.deltaRtoSpeed) - (speedL_kmh * 10);
#endif

    /* update speed and apply crosstalk */
    pwmL += ( speedDelta * cfgPaddle.pwmMultiplier * deltaMillis );
    pwmR += ( speedDelta * cfgPaddle.pwmMultiplier * deltaMillis * cfgPaddle.crosstalkLR );
    if(INPUT_PADDELEC_DEBUG) COM[DEBUG_COM]->print("g2L ");
#ifdef INPUT_PADDELEC
    if(INPUT_PADDELEC_DEBUG) COM[DEBUG_COM]->printf("%6i %6i %6i ",(int)((gametrak2.r - gametrak2.r_last) * cfgPaddle.deltaRtoSpeed), (int)speedL_kmh, (int)speedDelta);
#elif INPUT_PADDELECIMU
    if(INPUT_PADDELEC_DEBUG) COM[DEBUG_COM]->printf("%6i %6i %6i ",(int)(imu.gz * cfgPaddle.deltaRtoSpeed), (int)speedL_kmh, (int)speedDelta);
#endif


  } else if(paddleAngle <  -cfgPaddle.paddleAngleThreshold) {    // gametrak1 side of paddle is down

    /* get speed difference between paddle and "water". Paddling slower than current speed should slow down. */
#ifdef INPUT_PADDELEC
    double speedDelta = ((gametrak1.r - gametrak1.r_last) * cfgPaddle.deltaRtoSpeed) - speedR_kmh;
#elif INPUT_PADDELECIMU
    double speedDelta = (imu.gz * cfgPaddle.deltaRtoSpeed) - (speedR_kmh * 10);
#endif

    /* update speed and apply crosstalk */
    pwmR += ( speedDelta * cfgPaddle.pwmMultiplier * deltaMillis );
    pwmL += ( speedDelta * cfgPaddle.pwmMultiplier * deltaMillis * cfgPaddle.crosstalkLR );
    if(INPUT_PADDELEC_DEBUG) COM[DEBUG_COM]->print("g1R ");
#ifdef INPUT_PADDELEC
    if(INPUT_PADDELEC_DEBUG) COM[DEBUG_COM]->printf("%6i %6i %6i ",(int)((gametrak1.r - gametrak1.r_last) * cfgPaddle.deltaRtoSpeed), (int)speedR_kmh, (int)speedDelta);
#elif INPUT_PADDELECIMU
    if(INPUT_PADDELEC_DEBUG) COM[DEBUG_COM]->printf("%6i %6i %6i ",(int)(-imu.gz * cfgPaddle.deltaRtoSpeed), (int)speedR_kmh, (int)speedDelta);
#endif
  } else  {
    if(INPUT_PADDELEC_DEBUG) COM[DEBUG_COM]->print("___ ");
    if(INPUT_PADDELEC_DEBUG) COM[DEBUG_COM]->printf("%6i %6i %6i ", 0, 0, 0);
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

  if(imu.az > 80) {
    slowReset(steer, 0, 40);
    slowReset(pwm, 0, 40);
    return;
  }

//  plotterTempDouble[1] = imu.gz *0.1;
}

void Paddelec::debug(Stream &port)
{
#ifdef INPUT_PADDELEC
  gametrak1.debug(port);
  gametrak2.debug(port);
  port.printf("P %5i %5i %6.2f ", gametrak1.r - gametrak1.r_last, gametrak2.r - gametrak2.r_last, gametrak1.getTheta_deg() - gametrak2.getTheta_deg());
#elif INPUT_PADDELECIMU
  imu.debug(port);//
//  port.printf("P %5i %5i %6.2f ", imu.gz, -imu.gz, gametrak1.getTheta_deg() - gametrak2.getTheta_deg());
#endif
}

void Paddelec::RLpwmToSteer(double &steer, double &pwm, double &pwmR, double &pwmL)
{
  pwm = (pwmR + pwmL) / 2;
  steer = pwmL - pwm;
}

void Paddelec::steerToRL(double &steer, double &pwm, double &pwmR, double &pwmL)
{
  pwmR = pwm - steer;
  pwmL = pwm + steer;
}

// Incrementally decrease variable
void Paddelec::slowReset(double &variable, double goal, double step) {
  if      ((variable - goal) > step) variable -= step;
  else if ((goal - variable) > step) variable += step;
  else                               variable  = goal;
}

#endif