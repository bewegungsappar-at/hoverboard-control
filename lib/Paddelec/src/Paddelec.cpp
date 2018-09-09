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

void Paddelec::update(int16_t &pwm, int16_t &steer) {
  int16_t pwmR=0, pwmL=0;
  gametrak1.update();
  gametrak2.update();

  /* convert from speed and steering to left and right wheel speed */
  steerToRLpwm(steer, pwm, pwmR, pwmL);

  /* simulate drag */
  pwmL = pwmL * (1.0 - cfgPaddle.drag);
  pwmR = pwmR * (1.0 - cfgPaddle.drag);
  /* old method 
  if     (pwmL < -cfgPaddle.drag) pwmL = pwmL + cfgPaddle.drag;
  else if(pwmL >  cfgPaddle.drag) pwmL = pwmL - cfgPaddle.drag;
  else                              pwmL = 0;
  
  if     (pwmR < -cfgPaddle.drag) pwmR = pwmR + cfgPaddle.drag;
  else if(pwmR >  cfgPaddle.drag) pwmR = pwmR - cfgPaddle.drag;
  else                              pwmR = 0;
  */

  /* Kajak tries to align itself straight */
  pwmL += (pwmR - pwmL) * cfgPaddle.realign;
  pwmR += (pwmL - pwmR) * cfgPaddle.realign;

  /* process paddle strokes */
  if((gametrak2.getZ_mm() - gametrak1.getZ_mm()) < -cfgPaddle.zDiffThreshold) {
    // gametrak2 side of paddel is down

    /* get speed difference between paddle and "water". Paddling slower than current speed should slow down. */
    double speedDelta = ((gametrak2.r - gametrak2.r_last) * cfgPaddle.deltaRtoPWM) - pwmL;
pwmL += speedDelta * 4;
//    pwmL += (int16_t) ( ( speedDelta * speedDelta ) * cfgPaddle.speedMultiplier );
//    pwmR += (int16_t) ( ( speedDelta * cfgPaddle.speedMultiplier * cfgPaddle.crosstalkLR ) * ( speedDelta * cfgPaddle.speedMultiplier * cfgPaddle.crosstalkLR ) );


  } else if((gametrak2.getZ_mm() - gametrak1.getZ_mm()) > cfgPaddle.zDiffThreshold) {
    // gametrak1 side of paddel is down
    
    /* get speed difference between paddle and "water". Paddling slower than current speed should slow down. */
    double speedDelta = ((gametrak1.r - gametrak1.r_last) * cfgPaddle.deltaRtoPWM) - pwmR;

pwmR += speedDelta * 3;
//    pwmR += (int16_t) ( ( speedDelta * speedDelta ) * ( speedDelta * cfgPaddle.speedMultiplier ) );
//    pwmL += (int16_t) ( ( speedDelta * cfgPaddle.speedMultiplier * cfgPaddle.crosstalkLR ) * ( speedDelta * cfgPaddle.speedMultiplier * cfgPaddle.crosstalkLR ) );
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
  port.printf("P %5i %5i %5i ", gametrak1.r - gametrak1.r_last, gametrak2.r - gametrak2.r_last, gametrak2.getZ_mm() - gametrak1.getZ_mm());
}

void Paddelec::RLpwmToSteer(int16_t &steer, int16_t &pwm, int16_t &pwmR, int16_t &pwmL)
{
  pwm = (pwmR + pwmL) / 2;
  steer = pwmL - pwm;
}

void Paddelec::steerToRLpwm(int16_t &steer, int16_t &pwm, int16_t &pwmR, int16_t &pwmL)
{
  pwmR = pwm - steer;
  pwmL = pwm + steer;
}