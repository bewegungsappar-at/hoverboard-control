
#include "Paddelec.h"

#if defined(INPUT_PADDELECIMU) || defined(INPUT_PADDELEC)




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
  double paddleAngle = -imu.pitchangle();
#endif

  Serial.print("PD: ");

  /* process paddle strokes */
  if       (paddleAngle > cfgPaddle.paddleAngleThreshold) {    // gametrak2 side of paddle is down
 
    /* get speed difference between paddle and "water". Paddling slower than current speed should slow down. */
#ifdef INPUT_PADDELEC
    double speedDelta = ((gametrak2.r - gametrak2.r_last) * cfgPaddle.deltaRtoSpeed) - speedL_kmh;
#elif INPUT_PADDELECIMU
    double speedDelta = (imu.gz * cfgPaddle.deltaRtoSpeed) - speedL_kmh;
#endif

    /* update speed and apply crosstalk */
    pwmL += ( speedDelta * cfgPaddle.pwmMultiplier * deltaMillis );
    pwmR += ( speedDelta * cfgPaddle.pwmMultiplier * deltaMillis * cfgPaddle.crosstalkLR );
    Serial.print("g2L ");
#ifdef INPUT_PADDELEC
    Serial.printf("%6i %6i %6i ",(int)((gametrak2.r - gametrak2.r_last) * cfgPaddle.deltaRtoSpeed), (int)speedL_kmh, (int)speedDelta);
#elif INPUT_PADDELECIMU
    Serial.printf("%6i %6i %6i ",(int)(imu.gz * cfgPaddle.deltaRtoSpeed), (int)speedL_kmh, (int)speedDelta);
#endif


  } else if(paddleAngle <  -cfgPaddle.paddleAngleThreshold) {    // gametrak1 side of paddle is down

    /* get speed difference between paddle and "water". Paddling slower than current speed should slow down. */
#ifdef INPUT_PADDELEC
    double speedDelta = ((gametrak1.r - gametrak1.r_last) * cfgPaddle.deltaRtoSpeed) - speedR_kmh;
#elif INPUT_PADDELECIMU
    double speedDelta = (-imu.gz * cfgPaddle.deltaRtoSpeed) - speedR_kmh;
#endif

    /* update speed and apply crosstalk */
    pwmR += ( speedDelta * cfgPaddle.pwmMultiplier * deltaMillis );
    pwmL += ( speedDelta * cfgPaddle.pwmMultiplier * deltaMillis * cfgPaddle.crosstalkLR );
    Serial.print("g1R ");
#ifdef INPUT_PADDELEC
    Serial.printf("%6i %6i %6i ",(int)((gametrak1.r - gametrak1.r_last) * cfgPaddle.deltaRtoSpeed), (int)speedR_kmh, (int)speedDelta);
#elif INPUT_PADDELECIMU
    Serial.printf("%6i %6i %6i ",(int)(-imu.gz * cfgPaddle.deltaRtoSpeed), (int)speedR_kmh, (int)speedDelta);
#endif
  } else  {
    Serial.print("___ ");
    Serial.printf("%6i %6i %6i ", 0, 0, 0);
  }
      
  // TODO: calculate paddle engagement by angle, not only difference of adc values
  // TODO: use paddle angle as strength multiplier

  /* convert from left and right wheel speed to speed and steering */
  RLpwmToSteer(steer, pwm, pwmR, pwmL);
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

#endif