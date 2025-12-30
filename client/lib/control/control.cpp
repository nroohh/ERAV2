#include <control.h>
#include <utils.h>
#include <sstream>

float PIDfactor = 1.0;

void Control::arm() {
  cmd = "arm";
  on(LED_PIN);

  Serial.println("initialize...");
  hnu.init();
  for (int i = 0; i < 4; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].write(90);
  }

  edf.setPeriodHertz(50);
  edf.attach(EDF_PIN, 1000, 2000);
  edf.writeMicroseconds(1000); 
  delay(2000);
  Serial.println("initialization complete");
  blink(LED_PIN, 50, 3);
  on(LED_PIN);
  delay(500);

  Serial.println("calibrating...");
  hnu.calibrate();
  Serial.println("calibration complete");
  off(LED_PIN);
  delay(500);

  edf.writeMicroseconds(1150); 
  Serial.println("armed");

  blink(LED_PIN, 50, 3);
}

void Control::prelaunch() {
  cmd = "prelaunch";
  on(LED_PIN);

  reset();

  Serial.println("launching...");


  cmd = "launch";
}


void Control::launch() {

  hnu.timerStart();
  hnu.update();

  // use for magnetometer calibration
  // Serial.print(hnu.mpu.magX(), 2); Serial.print(", ");
  // Serial.print(hnu.mpu.magY(), 2); Serial.print(", ");
  // Serial.println(hnu.mpu.magZ(), 2);

  // PID
  rotationX.pid(hnu.pitch, hnu.dt);
  rotationY.pid(hnu.roll, hnu.dt);
  rotationZ.pid(hnu.yaw, hnu.dt);
  altitude.pid(hnu.distance, hnu.dt);


  // Serial.print("Pitch: "); Serial.print(hnu.pitch); Serial.print(", Roll: "); Serial.print(hnu.roll); Serial.print(", Yaw: "); Serial.println(hnu.yaw);
  
  // ACTION
  // edf.writeMicroseconds(1500 + altitude.output * 500); // throttle control
  Serial.print("c: "); Serial.print(altitude.c); Serial.print(" tgt: "); Serial.print(altitude.tgt); Serial.print(" output: "); Serial.println(1150 + (6 + 2.5 * altitude.output) * 100);
  edf.writeMicroseconds(1150 + (6 + 2.5 * altitude.output) * 100); // throttle control
  // Serial.print(rotationX.output); Serial.print(", "); Serial.print(rotationY.output); Serial.print(", "); Serial.println(rotationZ.output);

  PIDfactor = 3.5/(6 + 2.5 * altitude.output);

  if (PIDfactor > 3.0) {
    PIDfactor = 3.0;
  }

  servos[1].write(90 + SERVO_OFFSETS[1] + constrain((((1 - Z2XY_WEIGHT) * rotationY.output) - ((Z2XY_WEIGHT) * (rotationZ.output))) * PIDfactor * SERVO_RANGE, -SERVO_RANGE, SERVO_RANGE));
  servos[2].write(90 + SERVO_OFFSETS[2] + constrain((((1 - Z2XY_WEIGHT) * rotationX.output) - ((Z2XY_WEIGHT) * (rotationZ.output))) * PIDfactor * SERVO_RANGE, -SERVO_RANGE, SERVO_RANGE));
  servos[0].write(90 + SERVO_OFFSETS[0] + constrain(((-1 * (1 - Z2XY_WEIGHT) * rotationX.output) - ((Z2XY_WEIGHT) * (rotationZ.output))) * PIDfactor * SERVO_RANGE, -SERVO_RANGE, SERVO_RANGE));
  servos[3].write(90 + SERVO_OFFSETS[3] + constrain(((-1 * (1 - Z2XY_WEIGHT) * rotationY.output) - ((Z2XY_WEIGHT) * (rotationZ.output))) * PIDfactor * SERVO_RANGE, -SERVO_RANGE, SERVO_RANGE));

  delay(20);

  hnu.timerEnd(true);
}

void Control::pause() {
  cmd = "pause";
  edf.writeMicroseconds(1000);
}

void Control::kill() {
  cmd = "kill";
  for (int i = 0; i < 4; i++) {
    servos[i].write(90);
    servos[i].detach();
  }
  edf.writeMicroseconds(1000);
  edf.detach();
  off(LED_PIN);
}

void Control::reset() {
  cmd = "reset";
  Serial.println("resetting integrals...");
  rotationX.i = 0;
  rotationY.i = 0;
  rotationZ.i = 0;
  altitude.i = 0;
}