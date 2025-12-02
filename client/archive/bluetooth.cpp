// #include <ESP32Servo.h>  // Use the ESP32Servo library

// Define the servo object
// Servo myServo;

// Define the pin connected to the servo
// int servoPin = 25;

// void setup() {
//   // Attach the servo to the specified pin
//   myServo.attach(servoPin);
//   Serial.begin(115200);  // Start the serial communication
//   Serial.println("Servo test starting...");
// }

// This example code is in the Public Domain (or CC0 licensed, at your  option.)
// By Evandro Copercini - 2018
//
// This example creates a bridge  between Serial and Classical Bluetooth (SPP)
// and also demonstrate that SerialBT  have the same functionalities of a normal Serial
//
// The code displays the  received data and controls the on board LED
//
// Connect your device to ESP32test
//  Use Serial Bluetooth Terminal app.

// this library forms part of the ESP32  Boards Manager file
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;  // Create Bluetooth serial object

String text = "";          // Variable for incoming text
int ONBOARD_LED = 2;       // GPIO2 is often the built-in LED
int ln = 0;

void setup() {
  Serial.begin(115200);      // Start serial monitor communication
  SerialBT.begin("ERATEST"); // Bluetooth device name
  Serial.println("The device started, now you can pair it with Bluetooth!");
  
  pinMode(ONBOARD_LED, OUTPUT);  // Set up onboard LED pin as output
  digitalWrite(ONBOARD_LED, LOW);
}

void loop() {
  // Mirror serial monitor input to Bluetooth
  if (Serial.available()) {
    ln = (int)micros();
    SerialBT.write(Serial.read());
    SerialBT.println(String(ln));
  }

  // Handle incoming Bluetooth data
  if (SerialBT.available()) {
    text = SerialBT.readStringUntil('\n');  // Read until newline
    ln = (int)micros();
    text.trim();                            // Remove any leading or trailing spaces
    Serial.println("Received via Bluetooth: " + text + String(ln));  // Print received text
  }

  delay(20);  // Small delay to make the loop more stable

  
}


// void loop() {
//   // Rotate the servo to 0 degrees
//   myServo.write(0);
//   Serial.println("Servo moved to 0 degrees");
//   delay(1000);  // Wait for 1 second

//   // Rotate the servo to 90 degrees
//   myServo.write(90);
//   Serial.println("Servo moved to 90 degrees");
//   delay(1000);  // Wait for 1 second

//   // Rotate the servo to 180 degrees
//   myServo.write(180);
//   Serial.println("Servo moved to 180 degrees");
//   delay(1000);  // Wait for 1 second
// }