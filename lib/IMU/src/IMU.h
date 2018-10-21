#ifndef IMU_H
#define IMU_H
#include <Arduino.h>

#include <ArduinoNunchuk.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

#define IMU_GNDPIN 14
#define IMU_VCCPIN 32 



class Imu : public ArduinoNunchuk
{
  public:
    void init() {
        #ifdef IMU_GNDPIN
            pinMode(IMU_GNDPIN,OUTPUT);
            digitalWrite(IMU_GNDPIN,LOW);
        #endif
        #ifdef IMU_VCCPIN
            pinMode(IMU_VCCPIN,OUTPUT);
            digitalWrite(IMU_VCCPIN,HIGH);
        #endif

        pinMode(12,OUTPUT);
        digitalWrite(12,LOW);
        pinMode(33,INPUT_PULLUP);

        pinMode(13,OUTPUT);


        delay(100);

        Wire.begin();

        // initialize serial communication
        // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
        // it's really up to you depending on your project)
    
        // initialize device
        Serial.println("Initializing I2C devices...");
        accelgyro.initialize();

        // verify connection
        Serial.println("Testing device connections...");
        Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

        // use the code below to change accel/gyro offset values
        /*
        Serial.println("Updating internal sensor offsets...");
        // -76	-2359	1688	0	0	0
        Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
        Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
        Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
        Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
        Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
        Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
        Serial.print("\n");
        accelgyro.setXGyroOffset(220);
        accelgyro.setYGyroOffset(76);
        accelgyro.setZGyroOffset(-85);
        Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
        Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
        Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
        Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
        Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
        Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
        Serial.print("\n");
        */
       update();
       ArduinoNunchuk::cButton = 0;
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

        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.print(gz); Serial.print("\t");

        cButton_last = cButton;
        zButton_last = zButton;

        ArduinoNunchuk::analogX = 0;
        ArduinoNunchuk::analogY = 0;
        ArduinoNunchuk::accelX = (ax >> 4); // -511;
        ArduinoNunchuk::accelY = (ay >> 4);  // -511;
        ArduinoNunchuk::accelZ = (az >> 4); // -511;
        ArduinoNunchuk::zButton = 0;
        ArduinoNunchuk::cButton = 1 - digitalRead(33);
        digitalWrite(13,ArduinoNunchuk::cButton);

        return error;
    }


    int  update(double &speed, double &steer) {
        update();
        return ArduinoNunchuk::update(speed, steer);
    }




















};

#endif