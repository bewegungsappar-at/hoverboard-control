#include <Arduino.h>
#include "Platooning.h"



void Platooning::update(double &pwm, double &steer) {
  gametrak1.update();

  if(gametrak1.getR_mm() < cfgPlatooning.rActivationThreshold_mm) {
    /* gametrak distance is short, probably not in use */

    /* do nothing. If pwm and steer were set before, do not change. */    

  } else if(gametrak1.getR_mm() < cfgPlatooning.rForwardThreshold_mm) {
    /* drive backwards */
    pwm =   limit(-cfgPlatooning.PWMbackwardLimit, (gametrak1.getR_mm() - cfgPlatooning.rForwardThreshold_mm) * cfgPlatooning.r_mm_backwardToPWM, 0);
    steer = limit(-cfgPlatooning.SteerLimit,       gametrak1.getPhi_deg() * cfgPlatooning.phi_deg_ToSteer, cfgPlatooning.SteerLimit);
  } else {
    /* drive forward */
    pwm =   limit(0, (gametrak1.getR_mm() - cfgPlatooning.rForwardThreshold_mm) * cfgPlatooning.r_mm_forwardToPWM, cfgPlatooning.PWMforwardLimit);
    steer = limit(-cfgPlatooning.SteerLimit,       gametrak1.getPhi_deg() * cfgPlatooning.phi_deg_ToSteer, cfgPlatooning.SteerLimit);
  }
}

void Platooning::debug(Stream &port) 
{
  gametrak1.debug(port);
//  port.printf("P %5i %5i %6.2f ", gametrak1.r - gametrak1.r_last, gametrak1.r - gametrak1.r_last, gametrak1.getTheta_deg() - gametrak1.getTheta_deg());
}