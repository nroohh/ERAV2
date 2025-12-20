#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Arduino.h>
#include <control.h>
#include <config.h>
#include <ESP32Servo.h>
#include <utils.h>
#include <hnu.h>
#include <pid.h>
#include <map>
#include <vector>

class Control {
public:
    std::string cmd = "";
    Servo edf;
    Servo servos[4];
    HNU hnu;
    Axis rotationX;
    Axis rotationY;
    Axis rotationZ;
    Axis angularAccel;
    Axis altitude;

    std::map<std::string, float*> settingsMap;
    std::map<std::string, float*> targetMap;
    std::map<std::string, std::function<void()>> funcMap;

    Control() {
        settingsMap["SERVO_RANGE"] = &SERVO_RANGE; // individual variables
        settingsMap["Z2XY_WEIGHT"] = &Z2XY_WEIGHT;
        settingsMap["SERVO_OFFSETS_0"] = &SERVO_OFFSETS[0]; // servo offsets
        settingsMap["SERVO_OFFSETS_1"] = &SERVO_OFFSETS[1];
        settingsMap["SERVO_OFFSETS_2"] = &SERVO_OFFSETS[2];
        settingsMap["SERVO_OFFSETS_3"] = &SERVO_OFFSETS[3];
        settingsMap["RX_PG"] = &rotationX.pg; // rotation x
        settingsMap["RX_IG"] = &rotationX.ig;
        settingsMap["RX_DG"] = &rotationX.dg;
        settingsMap["RY_PG"] = &rotationY.pg; // rotation y
        settingsMap["RY_IG"] = &rotationY.ig;
        settingsMap["RY_DG"] = &rotationY.dg;
        settingsMap["RZ_PG"] = &rotationZ.pg; // rotation z
        settingsMap["RZ_IG"] = &rotationZ.ig;
        settingsMap["RZ_DG"] = &rotationZ.dg;
        settingsMap["AA_PG"] = &angularAccel.pg; // angular acceleration
        settingsMap["AA_IG"] = &angularAccel.ig;
        settingsMap["AA_DG"] = &angularAccel.dg;
        settingsMap["AL_PG"] = &altitude.pg; // throttle
        settingsMap["AL_IG"] = &altitude.ig;
        settingsMap["AL_DG"] = &altitude.dg;

        targetMap["RX_TGT"] = &rotationX.tgt; 
        targetMap["RY_TGT"] = &rotationY.tgt;
        targetMap["RZ_TGT"] = &rotationZ.tgt;
        targetMap["AL_TGT"] = &altitude.tgt;

        funcMap["arm"] = [this]() { this->arm(); };
        funcMap["reset"] = [this]() { this->reset(); };
        funcMap["launch"] = [this]() { this->launch(); };
        funcMap["kill"] = [this]() { this->kill(); };

        rotationX = Axis(0, 0, 0, 0, 0); // current, p gain, i gain, d gain, target
        rotationY = Axis(0, 0, 0, 0, 0);
        rotationZ = Axis(0, 0, 0, 0, 0);
        angularAccel = Axis(0, 0, 0, 0, 0);
        altitude = Axis(0, 0, 0, 0, 0);
    }

    void arm();
    void launch();
    void kill();
    void reset();
};


#endif