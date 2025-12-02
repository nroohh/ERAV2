#ifndef HNU_H
#define HNU_H

#include <MPU9250_asukiaaa.h>
#include <Adafruit_VL53L1X.h>
#include <Adafruit_AHRS_Madgwick.h>
#include <Arduino.h>
#include <Wire.h>
#include <config.h>
#include <utils.h>

class HNU { // Hybrid Navigation Unit
    public:
        MPU9250_asukiaaa mpu;
        Adafruit_VL53L1X vl = Adafruit_VL53L1X();
        Adafruit_Madgwick filter;
        double dt = 0; // delta time in seconds
        double pitch = 0;
        double roll = 0;
        double yaw = 0;
        double distance = 0;
    
        HNU();
        void init();
        void update(); // timerStart and timerEnd has to be called
        void calibrate();
        void timerStart();
        void timerEnd(bool await);

    private:
        unsigned long ts; // timerstart
        unsigned long te = 100UL; // time elapsed
        double accel[3] = {0, 0, 0};
        double gyro[3] = {0, 0, 0};
        double magneto[3] = {0, 0, 0};
        double pYaw = 0, pPitch = 0, pRoll = 0;
        double cYaw = 0, cPitch = 0, cRoll = 0;

        void initMPU();
        void initTOF();
        void initFilter();
        void calibrateAccel();
        void calibrateGyro();
        void calibrateMag();
        void stabilizeFilter();
        void adjustAccel(float ax, float ay, float az);
        void adjustGyro(float gx, float gy, float gz);
        void adjustMagneto(float mx,float my,float mz);
};

#endif