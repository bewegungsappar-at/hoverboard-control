#pragma once
#include <Arduino.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "config.h"

#define IMU_DEBUG false

#ifdef DEBUG_PLOTTER
  #include <Plotter.h>
  extern Plotter plot;
#endif

//#define IMU_CPIN 23
//#define IMU_ZPIN 25
//#define IMU_GNDPIN 14
//#define IMU_VCCPIN 32

#define NUNCHUK_JOYSTICK_THRESHOLD_LOW   77 // calibration values above this are considered invalid
#define NUNCHUK_JOYSTICK_THRESHOLD_HIGH 177 // calibration values below this are considered invalid
#define NUNCHUK_JOYSTICK_STEER_MULT     0.5 // 0.8 too much
#define NUNCHUK_JOYSTICK_SPEED_MULT     1.0 // 0.5 way too slow
#define NUNCHUK_ACCEL_SPEED_ANGLE        60 // Pitch angle needed to reach full speed (60° with factor 0.5 was too slow)
#define NUNCHUK_ACCEL_STEER_ANGLE       100 // Pitch angle needed to reach full speed (90° with factor 0.8 was ok,little slow)


class Imu
{
  public:
    Imu()
    {
        #ifdef IMU_GNDPIN
            pinMode(IMU_GNDPIN,OUTPUT);
            digitalWrite(IMU_GNDPIN,LOW);
        #endif
        #ifdef IMU_VCCPIN
            pinMode(IMU_VCCPIN,OUTPUT);
            digitalWrite(IMU_VCCPIN,HIGH);
        #endif
        #ifdef IMU_CPIN
            pinMode(IMU_CPIN,INPUT_PULLUP);
        #endif
        #ifdef IMU_ZPIN
            pinMode(IMU_ZPIN,INPUT_PULLUP);
        #endif
        delay(100);
    }

    int analogX, analogX_min=127, analogX_zero=0, analogX_max=127;
    int analogY, analogY_min=127, analogY_zero=0, analogY_max=127;
    int accelX, accelX_start;
    int accelY, accelY_start;
    int accelZ, accelZ_start;
    int zButton, zButton_last;
    int cButton, cButton_last;
    double yaw_zero=0, pitch_zero=0, roll_zero=0;

    double rollangle()  { return (atan2(accelX, accelZ) * 180.0 / M_PI); }
    double pitchangle() { return (atan2(accelY, accelZ) * 180.0 / M_PI); }
    double yawangle()   { return (atan2(accelZ, accelX) * 180.0 / M_PI); } // TODO: Check if it is working..

    double scaleAngle(double angle, double factor)    {
      if(angle<-180.0) angle+=360.0;
      else if (angle>180.0) angle-=360.0;
      return angle*factor;
    }
    double limit(double min, double value, double max) {
      if(value<min) value = min;
      if(value>max) value = max;
      return value;
    }

    void debug(Stream &port)
    {
      port.print("N ");
      port.printf("%4i %4i | ", analogX, analogY);
      port.printf("%4i %4i %4i | ", accelX, accelY, accelZ);
      port.printf("%2i %2i  \n", zButton, cButton);
    }

    void init()
    {
#ifndef DEBUG_OLED
# ifdef TTGO
        Wire.begin(5,4);
# else
        Wire.begin();
# endif
#endif

        // initialize serial communication
        // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
        // it's really up to you depending on your project)

        // initialize device
        if(IMU_DEBUG) delay(3000);
        if(IMU_DEBUG) Serial.println("Initializing I2C devices...");
        accelgyro.initialize();

        // verify connection
        if(IMU_DEBUG) Serial.println("Testing device connections...");
        if(IMU_DEBUG) Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

        if(IMU_DEBUG) Serial.println("Internal sensor offsets...");

        if(IMU_DEBUG) Serial.print(accelgyro.getXAccelOffset()); if(IMU_DEBUG) Serial.print("\t"); // -76
        if(IMU_DEBUG) Serial.print(accelgyro.getYAccelOffset()); if(IMU_DEBUG) Serial.print("\t"); // -2359
        if(IMU_DEBUG) Serial.print(accelgyro.getZAccelOffset()); if(IMU_DEBUG) Serial.print("\t"); // 1688
        if(IMU_DEBUG) Serial.print(accelgyro.getXGyroOffset()); if(IMU_DEBUG) Serial.print("\t"); // 0
        if(IMU_DEBUG) Serial.print(accelgyro.getYGyroOffset()); if(IMU_DEBUG) Serial.print("\t"); // 0
        if(IMU_DEBUG) Serial.print(accelgyro.getZGyroOffset()); if(IMU_DEBUG) Serial.print("\t"); // 0
        if(IMU_DEBUG) Serial.print("\n");

        if(IMU_DEBUG) Serial.println("Config values...");
        if(IMU_DEBUG) Serial.print("digital low-pass filter configuration: "); if(IMU_DEBUG) Serial.println(accelgyro.getDLPFMode());
        accelgyro.setDLPFMode(MPU6050_DLPF_BW_5);
        if(IMU_DEBUG) Serial.print("digital low-pass filter configuration: "); if(IMU_DEBUG) Serial.println(accelgyro.getDLPFMode());
        if(IMU_DEBUG) Serial.print("gyroscope output rate divider: "); if(IMU_DEBUG) Serial.println(accelgyro.getRate());
        accelgyro.setRate(0);
        if(IMU_DEBUG) Serial.print("gyroscope output rate divider: "); if(IMU_DEBUG) Serial.println(accelgyro.getRate());
        if(IMU_DEBUG) Serial.print("full-scale gyroscope range: "); if(IMU_DEBUG) Serial.println(accelgyro.getFullScaleGyroRange());
        accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
        if(IMU_DEBUG) Serial.print("full-scale gyroscope range: "); if(IMU_DEBUG) Serial.println(accelgyro.getFullScaleGyroRange());
        if(IMU_DEBUG) Serial.print("sleep mode: "); if(IMU_DEBUG) Serial.println(accelgyro.getSleepEnabled());
        accelgyro.setSleepEnabled(false);
        if(IMU_DEBUG) Serial.print("sleep mode: "); if(IMU_DEBUG) Serial.println(accelgyro.getSleepEnabled());
        if(IMU_DEBUG) Serial.print("full-scale accelerometer range: "); if(IMU_DEBUG) Serial.println(accelgyro.getFullScaleAccelRange());
        if(IMU_DEBUG) Serial.print("high-pass filter configuration: "); if(IMU_DEBUG) Serial.println(accelgyro.getDHPFMode());
        // Datasheet see https://www.invensense.com/products/motion-tracking/6-axis/mpu-6050/


        update();
        cButton = 0;
        zButton = 0;

        // Calibrate Gyro
        accelgyro.setXGyroOffset(0);
        accelgyro.setYGyroOffset(0);
        accelgyro.setZGyroOffset(0);
        delay(100);
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        delay(100);
        accelgyro.setXGyroOffset(-gx/4);
        accelgyro.setYGyroOffset(-gy/4);
        accelgyro.setZGyroOffset(-gz/4);
    }

    // class default I2C address is 0x68
    // specific I2C addresses may be passed as a parameter here
    // AD0 low = 0x68 (default for InvenSense evaluation board)
    // AD0 high = 0x69
    MPU6050 accelgyro;
    //MPU6050 accelgyro(0x69); // <-- use for AD0 high

    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    void loopIMU() {
        // read raw accel/gyro measurements from device
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // these methods (and a few others) are also available
        //accelgyro.getAcceleration(&ax, &ay, &az);
        //accelgyro.getRotation(&gx, &gy, &gz);

        // display tab-separated accel/gyro x/y/z values
        Serial.print("a/g:\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.print(gz); Serial.print("\t");
    }


    int update() {
        int error = 0;

        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
/*
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.print(gz); Serial.print("\t");
*/
        cButton_last = cButton;
        zButton_last = zButton;

        analogX = 0;
        analogY = 0;
        accelX = -(ay >> 4); // -511;
        accelY = -(ax >> 4);  // -511;
        accelZ = -(az >> 4); // -511;
    #ifdef IMU_ZPIN
        zButton = 1 - digitalRead(25);
    #else
        zButton = 0;
    #endif
    #ifdef IMU_CPIN
        cButton = 1 - digitalRead(23);
    #else
        cButton = 0;
    #endif

        return error;
    }


    int update(volatile double &speed, volatile double &steer) {
//        update();
//        return ArduinoNunchuk::update(speed, steer);

  int error = 0;
  error = update();

  if(cButton && !zButton)
  /* acceleration control mode when cButton is pressed */
  {
    if(cButton_last != cButton)
    {
      /* use current position as zero */
      accelX_start = accelX;
      accelY_start = accelY;
      accelZ_start = accelZ;
      pitch_zero   = pitchangle();
      yaw_zero     = yawangle();
      roll_zero    = rollangle();
    }
    double newSteer = scaleAngle(rollangle()  - roll_zero , 1000.0 / NUNCHUK_ACCEL_STEER_ANGLE);
    double newSpeed = scaleAngle(pitchangle() - pitch_zero, 1000.0 / NUNCHUK_ACCEL_SPEED_ANGLE);

    //newSteer = steer + limit(-70.0, newSteer - steer, 70.0);
    //newSpeed = speed + limit(-70.0, newSpeed - speed, 70.0);

    steer = (steer * 0.5) + (0.5 * newSteer);
    speed = (speed * 0.5) + (0.5 * newSpeed);
  } else if (cButton && zButton)
  /* Joystick calibration mode when both buttons are pressed */
  {
    if((zButton_last != zButton) || (cButton_last != cButton))  // do calibration
    {
      /* revoke old calibration, set zero */
      analogX_zero = analogX;
      analogY_zero = analogY;
      analogX_min = 127;
      analogX_max = 127;
      analogY_min = 127;
      analogY_max = 127;
    } else {
      /* find extremes */
      if(analogX < analogX_min) analogX_min = analogX;
      if(analogY < analogY_min) analogY_min = analogY;
      if(analogX > analogX_max) analogX_max = analogX;
      if(analogY > analogY_max) analogY_max = analogY;
    }
    steer = 0;
    speed = 0;
  } else if(!cButton && zButton && !zButton_last && (rollangle() > 90.0 || rollangle() < -90.0) ) {
    Serial2.print("1234567"); // Insert padding byte into serial stream to realign serial buffer
  } else
  /* use Joystick as Input */
  {
    // check if calib is plausible
    if(analogX_min  < NUNCHUK_JOYSTICK_THRESHOLD_LOW  && analogX_max  > NUNCHUK_JOYSTICK_THRESHOLD_HIGH
    && analogY_min  < NUNCHUK_JOYSTICK_THRESHOLD_LOW  && analogY_max  > NUNCHUK_JOYSTICK_THRESHOLD_HIGH
    && analogX_zero > NUNCHUK_JOYSTICK_THRESHOLD_LOW  && analogX_zero < NUNCHUK_JOYSTICK_THRESHOLD_HIGH
    && analogY_zero > NUNCHUK_JOYSTICK_THRESHOLD_LOW  && analogY_zero < NUNCHUK_JOYSTICK_THRESHOLD_HIGH)
    {
      if((analogX - analogX_zero)>0) steer = (analogX - analogX_zero) * 1000.0/(analogX_max - analogX_zero) * NUNCHUK_JOYSTICK_STEER_MULT;
      else                           steer = (analogX - analogX_zero) * 1000.0/(analogX_zero - analogX_min) * NUNCHUK_JOYSTICK_STEER_MULT;

      if((analogY - analogY_zero)>0) speed = (analogY - analogY_zero) * 1000.0/(analogY_max - analogY_zero) * NUNCHUK_JOYSTICK_SPEED_MULT;
      else                           speed = (analogY - analogY_zero) * 1000.0/(analogY_zero - analogY_min) * NUNCHUK_JOYSTICK_SPEED_MULT;

    } else {
      steer = 0;
      speed = 0;
    }
  }
  return error;

    }

};