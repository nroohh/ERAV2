#ifndef AXIS_H
#define AXIS_H

#include <Arduino.h>

class Axis {
    public:
        float c; // current value
        float pg; // p(proportional) gain
        float ig; // i(integral) gain
        float dg; // d(derivative) gain
        float tgt; // target value
        float e = 0; // error
        float pe = 0; // previous error
        float output; // pid output
        float i = 0; // accumilated error
        float d = 0; // change in  error

        Axis();
        Axis(float c, float pg, float ig, float dg, float tgt);
        void pid(float c, float dt);
};

#endif