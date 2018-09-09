#ifndef Paddelec_H
#define Paddelec_H

#include <Arduino.h>

// Only 2 Gametraks are possible, as the ESP32 ADC2 is not available when WIFI is used 
/*************************  Gametrak 1 *******************************/
//#define GAMETRAK1_VCCPIN    3V3      // Pin used to supply Power
#define GAMETRAK1_GNDPIN    25      // Pin used as GND
#define GAMETRAK1_RPIN      39      // wire length Pin Gametrak 1
#define GAMETRAK1_PHIPIN    36      // horizontal angle Pin Gametrak 1
#define GAMETRAK1_THETAPIN  34      // vertical angle Pin Gametrak 1
#define GAMETRAK1_PHI_REV      0    // Phi is inverted
#define GAMETRAK1_THETA_REV    0    // Theta is inverted
/*************************  Gametrak 2 *******************************/
#define GAMETRAK2_VCCPIN    27      // Pin used to supply Power
#define GAMETRAK2_GNDPIN    26      // Pin used as GND
#define GAMETRAK2_RPIN      32      // wire length Pin Gametrak 2
#define GAMETRAK2_PHIPIN    35      // horizontal angle Pin Gametrak 2
#define GAMETRAK2_THETAPIN  33      // vertical angle Pin Gametrak 2
#define GAMETRAK2_PHI_REV      1    // Phi is inverted
#define GAMETRAK2_THETA_REV    1    // Theta is inverted

#define ADC_MAXIMUM 4096                // 12 bit ADC
#define GAMETRAK_DEGREE_MAX_deg 80      // Gametrak has an opening angle of 80 deg.
#define GAMETRAK_DEGREE_MAX_rad 1.39626 // Gametrak has an opening angle of 80 deg. In rad: 80°/180°*pi = 1.39626 rad
#define GAMETRAK_LENGTH_MAX_mm  2900    // Measured 2.87m, 2.91m, 2.93m, 2.97m. So we assume 2.9m from now on. 

class Gametrak
{
  public:
    uint16_t r = 0;
    uint16_t r_last = 0;
    uint16_t theta = 0;
    uint16_t phi = 0;

    void update();
    Gametrak(uint8_t pinR, uint8_t pinPhi, uint8_t pinTheta, bool invPhi, bool invTheta) 
    {
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

class Paddelec 
{
  public:

    Gametrak gametrak1 = Gametrak(GAMETRAK1_RPIN, GAMETRAK1_PHIPIN, GAMETRAK1_THETAPIN, GAMETRAK1_PHI_REV, GAMETRAK1_THETA_REV);
    Gametrak gametrak2 = Gametrak(GAMETRAK2_RPIN, GAMETRAK2_PHIPIN, GAMETRAK2_THETAPIN, GAMETRAK2_PHI_REV, GAMETRAK2_THETA_REV);

    struct PaddelecConfig {
      int16_t zDiffThreshold;
      double pwmMultiplier;
      double crosstalkLR;
      double realign;
      double deltaRtoPWM;
      double drag;
      uint16_t pwmLimit;
      uint16_t steerLimit;
    };
    PaddelecConfig cfgPaddle;
    Paddelec()
    {
      switchOn();
      cfgPaddle.zDiffThreshold      = 300;                   // activation angle threshold of paddle. Below threshold, paddle is not enganged and paddelec is freewheeling.
      cfgPaddle.deltaRtoPWM         = 1.0;                   // conversion factor between Gametrak radius change and speed      
      cfgPaddle.pwmMultiplier       = 1.0;                   // effect of paddle stroke to speed
      cfgPaddle.crosstalkLR         = 0;                     // multiplier for steering
      cfgPaddle.realign             = cfgPaddle.crosstalkLR; // paddelc tries to go straight forward
      cfgPaddle.drag                = 0.0;                   // drag/water resistance
    }
    void update(int16_t &pwm, int16_t &steer);
    void debug(Stream &port);
    
  private: 
    void switchOn()  // TODO Integrate into Gametrak class
    {
      #ifdef GAMETRAK1_GNDPIN
        pinMode(GAMETRAK1_GNDPIN,OUTPUT);
        digitalWrite(GAMETRAK1_GNDPIN,LOW);
      #endif
      #ifdef GAMETRAK1_VCCPIN
        pinMode(GAMETRAK1_VCCPIN,OUTPUT);
        digitalWrite(GAMETRAK1_VCCPIN,HIGH);
      #endif
      #ifdef GAMETRAK2_GNDPIN
        pinMode(GAMETRAK2_GNDPIN,OUTPUT);
        digitalWrite(GAMETRAK2_GNDPIN,LOW);
      #endif
      #ifdef GAMETRAK2_GNDPIN
        pinMode(GAMETRAK2_VCCPIN,OUTPUT);
        digitalWrite(GAMETRAK2_VCCPIN,HIGH);
      #endif
    }
    
    void switchOff() // TODO Integrate into Gametrak class
    {
      #ifdef GAMETRAK1_GNDPIN
        pinMode(GAMETRAK1_GNDPIN,INPUT); // High Z
      #endif
      #ifdef GAMETRAK1_VCCPIN
        pinMode(GAMETRAK1_VCCPIN,INPUT); // High Z
      #endif
      #ifdef GAMETRAK2_GNDPIN
        pinMode(GAMETRAK2_GNDPIN,INPUT); // High Z
      #endif
      #ifdef GAMETRAK2_GNDPIN
        pinMode(GAMETRAK2_VCCPIN,INPUT); // High Z
      #endif
    }
    void RLpwmToSteer(int16_t &steer, int16_t &pwm, int16_t &pwmR, int16_t &pwmL);
    void steerToRLpwm(int16_t &steer, int16_t &pwm, int16_t &pwmR, int16_t &pwmL);
};

#endif