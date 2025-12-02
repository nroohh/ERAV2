#include <control.h>
#include <utils.h>
#include <sstream>

void Control::arm() {
  blink(LED_PIN, 50, 3);
  hnu.init();
  // hnu.calibrate();
}

void Control::launch() {
  blink(LED_PIN, 100, 1);
  hnu.timerStart();
  hnu.update();
  Serial.println(hnu.distance);
  delay(20);
  hnu.timerEnd(true);
}

void Control::kill() {
  blink(LED_PIN, 50, 10);
}


// void Control::arm() {
//   Serial.println("executing command: arm");
//   on(LED_PIN);

//   Serial.println("initializing...");
//   hnu.init();
//   for (int i = 0; i < 4; i++) {
//     servos[i].attach(SERVO_PINS[i]);
//     servos[i].write(90);
//   }
//   edf.setPeriodHertz(50);
//   edf.attach(EDF_PIN, 1000, 2000);
//   edf.writeMicroseconds(1000); 
//   delay(2000);
//   Serial.println("initialization complete");
//   blink(LED_PIN, 50, 3);
//   on(LED_PIN);
//   delay(500);

//   Serial.println("calibrating...");
//   hnu.calibrate();
//   Serial.println("calibration complete");
//   off(LED_PIN);
//   delay(500);

//   blink(LED_PIN, 50, 3);
// }

// void Control::launch() {
//   Serial.println("executing command: launch");
//   while (cmd == "launch") {
//     hnu.timerStart();

//     hnu.update();

//     // PID
//     rotationX.pid(hnu.pitch, hnu.dt);
//     rotationY.pid(hnu.roll, hnu.dt);
//     rotationZ.pid(hnu.yaw, hnu.dt);
    
//     // ACTION
//     edf.writeMicroseconds(1500 + altitude.output * 500); // throttle control
//     Serial.print(rotationX.output); Serial.print(", "); Serial.print(rotationY.output); Serial.print(", "); Serial.println(rotationZ.output);

//     servos[0].write(90 + SERVO_OFFSETS[0] + (((1 - Z2XY_WEIGHT) * rotationX.output) - ((Z2XY_WEIGHT) * rotationZ.output)) * SERVO_RANGE);
//     servos[1].write(90 + SERVO_OFFSETS[1] + ((-1 * (1 - Z2XY_WEIGHT) * rotationY.output) - ((Z2XY_WEIGHT) * rotationZ.output)) * SERVO_RANGE);
//     servos[3].write(90 + SERVO_OFFSETS[3] + ((-1 * (1 - Z2XY_WEIGHT) * rotationX.output) - ((Z2XY_WEIGHT) * rotationZ.output)) * SERVO_RANGE);
//     servos[2].write(90 + SERVO_OFFSETS[2] + (((1 - Z2XY_WEIGHT) * rotationY.output) - ((Z2XY_WEIGHT) * rotationZ.output)) * SERVO_RANGE);

//     delay(20);

//     hnu.timerEnd(true);
//   }
// }

// void Control::kill() {
//   Serial.println("executing command: kill");
//   for (int i = 0; i < 4; i++) {
//     servos[i].write(90);
//     servos[i].detach();
//   }
//   edf.writeMicroseconds(1000);
//   edf.detach();
//   off(LED_PIN);
// }
