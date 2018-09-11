#include <Arduino.h>
#include "Paddelec.h"



void Gametrak::update()
{
  r_last = r;
  r      = analogRead(pin_r);
  phi    = analogRead(pin_phi);
  theta  = analogRead(pin_theta);

  if(inv_phi) phi      = 4096 - phi;
  if(inv_theta) theta  = 4096 - theta;
}

void Paddelec::update(int16_t &pwm, int16_t &steer, double &actualSpeed_kmh, double &actualSteer_kmh) {
  int16_t pwmR     =0, pwmL     =0;
  double  speedR_kmh=0, speedL_kmh=0;
  gametrak1.update();
  gametrak2.update();

  /* convert from speed and steering to left and right wheel speed */
  steerToRL(steer,           pwm,             pwmR,       pwmL);
  steerToRL(actualSteer_kmh, actualSpeed_kmh, speedR_kmh, speedL_kmh);

  /* simulate drag */
  /* decrease pwm by a factor each time function is called */
  pwmL *= 1.0 - cfgPaddle.drag;
  pwmR *= 1.0 - cfgPaddle.drag;

  /* Kajak tries to align itself straight */
  pwmL += (pwmR - pwmL) * cfgPaddle.realign;
  pwmR += (pwmL - pwmR) * cfgPaddle.realign;


  float paddleAngle = gametrak1.getTheta_deg() - gametrak2.getTheta_deg();

  /* process paddle strokes */
  if       (paddleAngle > cfgPaddle.paddleAngleThreshold) {    // gametrak2 side of paddle is down
 
    /* get speed difference between paddle and "water". Paddling slower than current speed should slow down. */
    double speedDelta = ((gametrak2.r - gametrak2.r_last) * cfgPaddle.deltaRtoSpeed) - speedL_kmh;

    /* update speed and apply crosstalk */
    pwmL += (int16_t) ( speedDelta * cfgPaddle.pwmMultiplier );
    pwmR += (int16_t) ( speedDelta * cfgPaddle.pwmMultiplier * cfgPaddle.crosstalkLR );
    Serial.print("g2L ");
    Serial.printf("%6i %6i %6i ",(int)((gametrak2.r - gametrak2.r_last) * cfgPaddle.deltaRtoSpeed), (int)speedL_kmh, (int)speedDelta);


  } else if(paddleAngle <  -cfgPaddle.paddleAngleThreshold) {    // gametrak1 side of paddle is down

    /* get speed difference between paddle and "water". Paddling slower than current speed should slow down. */
    double speedDelta = ((gametrak1.r - gametrak1.r_last) * cfgPaddle.deltaRtoSpeed) - speedR_kmh;

    /* update speed and apply crosstalk */
    pwmR += (int16_t) ( speedDelta * cfgPaddle.pwmMultiplier );
    pwmL += (int16_t) ( speedDelta * cfgPaddle.pwmMultiplier * cfgPaddle.crosstalkLR );
    Serial.print("g1R ");
    Serial.printf("%6i %6i %6i ",(int)((gametrak1.r - gametrak1.r_last) * cfgPaddle.deltaRtoSpeed), (int)speedR_kmh, (int)speedDelta);
  } else 
  {
    Serial.print("nix ");
    Serial.printf("%6i %6i %6i ", 0, 0, 0);
  }
      
  // TODO: calculate paddle engagement by angle, not only difference of adc values
  // TODO: use paddle angle as strength multiplier

  /* convert from left and right wheel speed to speed and steering */
  RLpwmToSteer(steer, pwm, pwmR, pwmL);
}

void Gametrak::debug(Stream &port) 
{
//  port.printf("G %5i %5i %5i ", r, phi, theta);
  port.printf("G %5i %6.2f %6.2f ", getR_mm(), getPhi_deg(), getTheta_deg());
}
void Paddelec::debug(Stream &port) 
{
  gametrak1.debug(port);
  gametrak2.debug(port);
  port.printf("P %5i %5i %6.2f ", gametrak1.r - gametrak1.r_last, gametrak2.r - gametrak2.r_last, gametrak1.getTheta_deg() - gametrak2.getTheta_deg());
}

void Paddelec::RLpwmToSteer(int16_t &steer, int16_t &pwm, int16_t &pwmR, int16_t &pwmL)
{
  pwm = (pwmR + pwmL) / 2;
  steer = pwmL - pwm;
}

void Paddelec::steerToRL(int16_t &steer, int16_t &pwm, int16_t &pwmR, int16_t &pwmL)
{
  pwmR = pwm - steer;
  pwmL = pwm + steer;
}

void Paddelec::steerToRL(double &steer, double &pwm, double &pwmR, double &pwmL)
{
  pwmR = pwm - steer;
  pwmL = pwm + steer;
}