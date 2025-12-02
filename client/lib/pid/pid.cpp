#include "pid.h"

Axis::Axis() : c(0), pg(0), ig(0), dg(0), tgt(0) {}
Axis::Axis(float c, float pg, float ig, float dg, float tgt) 
    : c(c), pg(pg), ig(ig), dg(dg), tgt(tgt) {}

void Axis::pid(float value, float dt) {
    c = value;
    output = 0;

    e = tgt - c;
    i += e * dt;
    d = (e - pe) / dt;

    // pid values
    float pv = pg * e; 
    float iv = ig * i;
    float dv = dg * d;

    output = pv + iv + dv;
    pe = e;

    if (abs(output) > 1) { // limits output [-1, 1]
        output = output / abs(output);
    }
}