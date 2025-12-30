#include <Arduino.h>
#include <config.h>
#include <blue.h>
#include <utils.h>

using namespace std;

void setup() {
    pinMode(INPUT_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    blink(LED_PIN, 50, 3);
    off(LED_PIN);
    Serial.begin(115200);
    initBLE();
    Serial.println("bluetooth initialized");
    on(LED_PIN);
}

void loop() {
    if (control == nullptr) {
        delay(100);
        return;
    }

    
    if (control->cmd == "launch") {
        control->launch();
    } else {
        delay(100);
    }
}

// #include <Arduino.h>
// #include <ESP32Servo.h>

// int SERVO_PINS[4] = {14, 27, 25, 26};
// Servo servos[4];
// int SERVO_COUNT = 4;

// void setup() {
//   Serial.begin(115200);

//   // Attach all servos
//   for (int i = 0; i < SERVO_COUNT; i++) {
//     servos[i].setPeriodHertz(50);
//     servos[i].attach(SERVO_PINS[i], 500, 2400);
//     servos[i].write(90);  // default position
//   }

//   Serial.println("Commands:");
//   Serial.println("  set <index> <angle>");
//   Serial.println("  setall <angle>");
//   Serial.println("  calibrate");
// }

// // Move all servos to a specific angle
// void moveAllServos(int angle) {
//   for (int i = 0; i < SERVO_COUNT; i++) {
//     servos[i].write(angle);
//   }
// }

// // Perform full sweep calibration
// void calibrationSequence() {
//   Serial.println("Starting calibration...");

//   // Sweep up 30 → 120
//   for (int angle = 30; angle <= 120; angle += 5) {
//     moveAllServos(angle);
//     delay(50);
//   }

//   // Sweep back 120 → 90
//   for (int angle = 120; angle >= 90; angle -= 5) {
//     moveAllServos(angle);
//     delay(50);
//   }

//   Serial.println("Calibration done.");
// }

// void handleSetCommand(String input) {
//   int firstSpace = input.indexOf(' ');
//   int secondSpace = input.indexOf(' ', firstSpace + 1);

//   if (firstSpace == -1 || secondSpace == -1) {
//     Serial.println("Invalid format. Use: set <index> <angle>");
//     return;
//   }

//   int index = input.substring(firstSpace + 1, secondSpace).toInt();
//   int angle = input.substring(secondSpace + 1).toInt();

//   if (index < 0 || index >= SERVO_COUNT) {
//     Serial.println("Servo index out of range (0–3).");
//     return;
//   }
//   if (angle < 0 || angle > 180) {
//     Serial.println("Angle must be 0–180.");
//     return;
//   }

//   servos[index].write(angle);

//   Serial.print("Servo ");
//   Serial.print(index);
//   Serial.print(" set to ");
//   Serial.println(angle);
// }

// void loop() {
//   if (Serial.available()) {
//     String cmd = Serial.readStringUntil('\n');
//     cmd.trim();

//     if (cmd.startsWith("setall")) {
//       int angle = cmd.substring(7).toInt();
//       if (angle >= 0 && angle <= 180) {
//         moveAllServos(angle);
//         Serial.print("All servos set to ");
//         Serial.println(angle);
//       } else {
//         Serial.println("Angle must be 0–180.");
//       }

//     } else if (cmd.startsWith("set")) {
//       handleSetCommand(cmd);

//     } else if (cmd == "calibrate") {
//       calibrationSequence();

//     } else {
//       Serial.println("Unknown command.");
//     }
//   }
// }
