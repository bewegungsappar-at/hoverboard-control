#include <Arduino.h>
#include "Gametrak.h"



void Gametrak::update() {
  r_last = r;
  r      = analogRead(pin_r);
  phi    = analogRead(pin_phi);
  theta  = analogRead(pin_theta);

  if(inv_phi) phi      = 4096 - phi;
  if(inv_theta) theta  = 4096 - theta;
}

void Gametrak::debug(Stream &port) {
//  port.printf("G %5i %5i %5i ", r, phi, theta);
  port.printf("G %5i %6.2f %6.2f ", getR_mm(), getPhi_deg(), getTheta_deg());
}