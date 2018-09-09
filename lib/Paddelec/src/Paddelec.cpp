#include <Arduino.h>
#include "Paddelec.h"



void Gametrak::update()
{
  r_last = r;
  r      = analogRead(pin_r);
  phi    = analogRead(pin_phi);
  theta  = analogRead(pin_theta);

  if(inv_phi) phi    = 4096 - phi;
  if(inv_theta) theta  = 4096 - theta;
}

void Paddelec::update(int16_t &pwm, int16_t &steer, int16_t &actualSpeed_mh, int16_t &actualSteer_mh) {
  int16_t pwmR     =0, pwmL     =0;
  int16_t speedR_mh=0, speedL_mh=0;
  gametrak1.update();
  gametrak2.update();

  /* convert from speed and steering to left and right wheel speed */
  steerToRL(steer,          pwm,            pwmR,      pwmL);
  steerToRL(actualSteer_mh, actualSpeed_mh, speedR_mh, speedL_mh);

  /* simulate drag */
  /* decrease pwm by a factor each time function is called */
  pwmL *= 1.0 - cfgPaddle.drag;
  pwmR *= 1.0 - cfgPaddle.drag;

  /* Kajak tries to align itself straight */
  pwmL += (pwmR - pwmL) * cfgPaddle.realign;
  pwmR += (pwmL - pwmR) * cfgPaddle.realign;


  float paddleAngle = gametrak1.getTheta_deg() - gametrak2.getTheta_deg();

  /* process paddle strokes */
  if       (paddleAngle < -cfgPaddle.paddleAngleThreshold) {    // gametrak2 side of paddle is down
 
    /* get speed difference between paddle and "water". Paddling slower than current speed should slow down. */
    double speedDelta = ((gametrak2.r - gametrak2.r_last) * cfgPaddle.deltaRtoSpeed) - speedL_mh;

    /* update speed and apply crosstalk */
    pwmL += (int16_t) ( speedDelta * cfgPaddle.pwmMultiplier );
    pwmR += (int16_t) ( speedDelta * cfgPaddle.pwmMultiplier * cfgPaddle.crosstalkLR );


  } else if(paddleAngle >  cfgPaddle.paddleAngleThreshold) {    // gametrak1 side of paddle is down

    /* get speed difference between paddle and "water". Paddling slower than current speed should slow down. */
    double speedDelta = ((gametrak1.r - gametrak1.r_last) * cfgPaddle.deltaRtoSpeed) - speedR_mh;

    /* update speed and apply crosstalk */
    pwmR += (int16_t) ( speedDelta * cfgPaddle.pwmMultiplier );
    pwmL += (int16_t) ( speedDelta * cfgPaddle.pwmMultiplier * cfgPaddle.crosstalkLR );
  } 
      
  // TODO: calculate paddle engagement by angle, not only difference of adc values
  // TODO: use paddle angle as strength multiplier

  /* convert from left and right wheel speed to speed and steering */
  RLpwmToSteer(steer, pwm, pwmR, pwmL);
}

void Gametrak::debug(Stream &port) 
{
//  port.printf("G %5i %5i %5i ", r, phi, theta);
  port.printf("G %5i %4.2f %4.2f ", getR_mm(), getPhi_deg(), getTheta_deg());
}
void Paddelec::debug(Stream &port) 
{
  gametrak1.debug(port);
  gametrak2.debug(port);
  port.printf("P %5i %5i %4.2f ", gametrak1.r - gametrak1.r_last, gametrak2.r - gametrak2.r_last, gametrak1.getTheta_deg() - gametrak2.getTheta_deg());
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