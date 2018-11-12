#pragma once
#include <Arduino.h>


#define ADC_MAXIMUM 4096                // 12 bit ADC
#define GAMETRAK_DEGREE_MAX_deg 80      // Gametrak has an opening angle of 80 deg.
#define GAMETRAK_DEGREE_MAX_rad 1.39626 // Gametrak has an opening angle of 80 deg. In rad: 80°/180°*pi = 1.39626 rad
#define GAMETRAK_LENGTH_MAX_mm  2900    // Measured 2.87m, 2.91m, 2.93m, 2.97m. So we assume 2.9m from now on. 

class Gametrak {
  public:
    uint16_t r = 0;
    uint16_t r_last = 0;
    uint16_t theta = 0;
    uint16_t phi = 0;

    void update();
    Gametrak(uint8_t pinR, uint8_t pinPhi, uint8_t pinTheta, bool invPhi, bool invTheta) {
      pin_r = pinR; 
      pin_phi = pinPhi; 
      pin_theta = pinTheta; 
      inv_phi = invPhi; 
      inv_theta = invTheta;
    }
    void debug(Stream &port);


    int getR_mm() { return (int) ((float) r / (float) ADC_MAXIMUM * (float) GAMETRAK_LENGTH_MAX_mm); }
    int getY_mm() { return sin( getPhi_rad()   ) * getR_mm(); } // TODO assumed 2D..
    int getZ_mm() { return sin( getTheta_rad() ) * getR_mm(); } // TODO assumed 2D..

    float getTheta_deg() { return  (theta - (ADC_MAXIMUM/2))  / (float) ADC_MAXIMUM * GAMETRAK_DEGREE_MAX_deg; }
    float getTheta_rad() { return  (theta - (ADC_MAXIMUM/2))  / (float) ADC_MAXIMUM * GAMETRAK_DEGREE_MAX_rad; }
    float getPhi_deg()   { return  (phi   - (ADC_MAXIMUM/2))  / (float) ADC_MAXIMUM * GAMETRAK_DEGREE_MAX_deg; }
    float getPhi_rad()   { return  (phi   - (ADC_MAXIMUM/2))  / (float) ADC_MAXIMUM * GAMETRAK_DEGREE_MAX_rad; }

  private:
    uint8_t pin_r;
    uint8_t pin_phi;
    uint8_t pin_theta;
    bool inv_phi;
    bool inv_theta;
};