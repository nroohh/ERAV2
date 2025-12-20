#include "pid.h"

Axis::Axis() : c(0), pg(0), ig(0), dg(0), tgt(0) {}
Axis::Axis(float c, float pg, float ig, float dg, float tgt) 
    : c(c), pg(pg), ig(ig), dg(dg), tgt(tgt) {}

void Axis::pid(float value, float dt) {
    c = value;
    output = 0;

    e = tgt - c; 
    float pv = pg * e; 

    d = (e - pe) / dt;
    float dv = dg * d;
    
    float po = pv + dv; // potential output without integral
    float di = e * dt; // integral change
    float tpo = po + ig * (i + di); // toatal potential output
    bool is_saturating = false;
    
    if (tpo > 1.0) {
        if (e > 0) {
            is_saturating = true;
        }
    } else if (tpo < -1.0) {
        if (e < 0) {
            is_saturating = true;
        }
    }

    if (!is_saturating) {
        i += di;
    }
    
    float iv = ig * i;

    output = pv + iv + dv;

    if (output > 1.0) {
        output = 1.0;
    } else if (output < -1.0) {
        output = -1.0;
    }

    pe = e;
}