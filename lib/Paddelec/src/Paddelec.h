#ifndef Paddelec_H
#define Paddelec_H

#include <Arduino.h>

// Only 2 Gametraks are possible, as the ESP32 ADC2 is not available when WIFI is used 
/*************************  Gametrak 1 *******************************/
//#define GAMETRAK1_VCCPIN    3V3      // Pin used to supply Power
#define GAMETRAK1_GNDPIN    27      // Pin used as GND
#define GAMETRAK1_RPIN      39      // wire length Pin Gametrak 1
#define GAMETRAK1_PHIPIN    36      // horizontal angle Pin Gametrak 1
#define GAMETRAK1_THETAPIN  34      // vertical angle Pin Gametrak 1
#define GAMETRAK1_PHI_REV      0    // Phi is inverted
#define GAMETRAK1_THETA_REV    0    // Theta is inverted
/*************************  Gametrak 2 *******************************/
#define GAMETRAK2_VCCPIN    25      // Pin used to supply Power
#define GAMETRAK2_GNDPIN    26      // Pin used as GND
#define GAMETRAK2_RPIN      32      // wire length Pin Gametrak 2
#define GAMETRAK2_PHIPIN    35      // horizontal angle Pin Gametrak 2
#define GAMETRAK2_THETAPIN  33      // vertical angle Pin Gametrak 2
#define GAMETRAK2_PHI_REV      1    // Phi is inverted
#define GAMETRAK2_THETA_REV    1    // Theta is inverted

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
      int16_t thetaDiffThreshold;
      float speedMultiplier;
      float steerMultiplier;
      int16_t drag;
      uint16_t speedLimit;
      uint16_t steerLimit;
    };
    PaddelecConfig cfgPaddle;
    Paddelec()
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

      cfgPaddle.thetaDiffThreshold  = 1000; // activation angle threshold of paddle. Below threshold, paddle is not enganged and paddelec is freewheeling.
      cfgPaddle.speedMultiplier     = 10.0; // multiplier for speed
      cfgPaddle.steerMultiplier     = 0.5;  // multiplier for steering
      cfgPaddle.drag                = 1;    // drag/water resistance
      cfgPaddle.speedLimit          = 500;  // speed limit
      cfgPaddle.steerLimit          = 100;  // steering limit
    }
    void update(int16_t &speed, int16_t &steer);
    void debug(Stream &port);
    
};

#endif