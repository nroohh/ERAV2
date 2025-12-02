#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

void hault(int pin); // wait for button press
void blink(int pin, int dt, int count); // wait for button press
void on(int pin);
void off(int pin);

#endif