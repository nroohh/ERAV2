#include "utils.h"

void hault(int pin) {
  while (digitalRead(pin) == HIGH) {} // wait until pressed (LOW)
  while (digitalRead(pin) == LOW) {} // wait until released (HIGH)
}

void blink(int pin, int dt, int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(pin, HIGH);
    delay(dt);
    digitalWrite(pin, LOW);
    delay(dt);
  }
}

void on(int pin) {
  digitalWrite(pin, HIGH);
}

void off(int pin) {
  digitalWrite(pin, LOW);
}