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
  gametrak1.update();
  gametrak2.update();
  if(gametrak1.theta-gametrak2.theta < -cfgPaddle.thetaDiffThreshold) {
    // gametrak2 side of paddel is down
    speed = (int16_t)((gametrak1.r - gametrak1.r_last) * cfgPaddle.speedMultiplier);
    steer = speed * cfgPaddle.steerMultiplier;
  } else if(gametrak1.theta-gametrak2.theta > cfgPaddle.thetaDiffThreshold) {
    // gametrak1 side of paddel is down
    speed = (gametrak2.r - gametrak2.r_last) * cfgPaddle.speedMultiplier;
    steer = speed * -cfgPaddle.steerMultiplier;
  } else {
    // paddel is not engaged
    // decelerate slowly
    if(speed < -cfgPaddle.drag) {
      speed = speed + cfgPaddle.drag;
    } else if(speed > cfgPaddle.drag) {
      speed = speed - cfgPaddle.drag;
    } else {
      speed = 0;
    }
    steer = 0;
  }
  
  /* limit motor speed */
  if(speed < -cfgPaddle.speedLimit) {
    speed = -cfgPaddle.speedLimit;
  } else if(speed > cfgPaddle.speedLimit) {
    speed = cfgPaddle.speedLimit;
  } 
  
  /* limit steering */
  if(steer < -cfgPaddle.steerLimit) {
    steer = -cfgPaddle.steerLimit;
  } else if(steer > cfgPaddle.steerLimit) {
    steer = cfgPaddle.steerLimit;
  } 

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