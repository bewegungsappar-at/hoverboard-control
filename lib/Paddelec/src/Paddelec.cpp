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

void Paddelec::update(int16_t &speed, int16_t &steer) {
  int16_t speedR=0, speedL=0;
  gametrak1.update();
  gametrak2.update();

  /* convert from speed and steering to left and right wheel speed */
  steerToSpeeds(steer, speed, speedR, speedL);

  /* simulate drag */
  speedL = speedL * (1.0 - cfgPaddle.drag);
  speedR = speedR * (1.0 - cfgPaddle.drag);
  /* old method 
  if     (speedL < -cfgPaddle.drag) speedL = speedL + cfgPaddle.drag;
  else if(speedL >  cfgPaddle.drag) speedL = speedL - cfgPaddle.drag;
  else                              speedL = 0;
  
  if     (speedR < -cfgPaddle.drag) speedR = speedR + cfgPaddle.drag;
  else if(speedR >  cfgPaddle.drag) speedR = speedR - cfgPaddle.drag;
  else                              speedR = 0;
  */

  /* Kajak tries to align itself straight */
  speedL += (speedR - speedL) * cfgPaddle.realign;
  speedR += (speedL - speedR) * cfgPaddle.realign;

  /* process paddle strokes */
  if(gametrak1.theta-gametrak2.theta < -cfgPaddle.thetaDiffThreshold) {
    // gametrak2 side of paddel is down

    /* get speed difference between paddle and "water". Paddling slower than current speed should slow down. */
    double speedDelta = ((gametrak1.r - gametrak1.r_last) * cfgPaddle.deltaRtoSpeed) - speedL;

    speedL += (int16_t) ( speedDelta * cfgPaddle.speedMultiplier );
    speedR += (int16_t) ( speedDelta * cfgPaddle.speedMultiplier * cfgPaddle.crosstalkLR );

  } else if(gametrak1.theta-gametrak2.theta > cfgPaddle.thetaDiffThreshold) {
    // gametrak1 side of paddel is down
    
    /* get speed difference between paddle and "water". Paddling slower than current speed should slow down. */
    double speedDelta = ((gametrak2.r - gametrak2.r_last) * cfgPaddle.deltaRtoSpeed) - speedR;

    speedR += (int16_t) ( speedDelta * cfgPaddle.speedMultiplier );
    speedL += (int16_t) ( speedDelta * cfgPaddle.speedMultiplier * cfgPaddle.crosstalkLR );

  } 
      
  // TODO: calculate paddle engagement by angle, not only difference of adc values
  // TODO: use paddle angle as strength multiplier

  /* convert from left and right wheel speed to speed and steering */
  speedsToSteer(steer, speed, speedR, speedL);
}

void Gametrak::debug(Stream &port) 
{
  port.printf("G %5i %5i %5i ", r, phi, theta);
}
void Paddelec::debug(Stream &port) 
{
  gametrak1.debug(port);
  gametrak2.debug(port);
  port.printf("P %5i %5i %5i ", gametrak1.r - gametrak1.r_last, gametrak2.r - gametrak2.r_last, gametrak1.theta-gametrak2.theta);
}

void Paddelec::speedsToSteer(int16_t &steer, int16_t &speed, int16_t &speedR, int16_t &speedL)
{
  speed = (speedR + speedL) / 2;
  steer = speedL - speed;
}

void Paddelec::steerToSpeeds(int16_t &steer, int16_t &speed, int16_t &speedR, int16_t &speedL)
{
  speedR = speed - steer;
  speedL = speed + steer;
}