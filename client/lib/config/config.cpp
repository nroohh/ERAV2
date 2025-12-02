#include "config.h"
#include <map>
#include <string>

std::string SERVICE_UUID        = "37683225-dfbc-4e96-a010-e9c708151478"; 
std::string COMMAND_CHARACTERISTIC_UUID = "b7b88cad-dff1-492c-ab98-2aab1c81da84"; // arm, launch, kill; used to send commands
std::string TARGET_CHARACTERISTIC_UUID = "4fe486b9-93fc-4333-a5af-b8dcea135cf3"; // rx, ry, rz, al; used to control the target values
std::string SETTINGS_CHARACTERISTIC_UUID = "1f8c64d3-1ca4-43c3-88f9-b4c8df79be5a"; // rx: {}, ry: {}, rz: {}, aa: {}, tt: {}; used to control PID gains

float SERVO_OFFSETS[4] = {12, 6, -12, -3};
float Z2XY_WEIGHT = 0.5;
float SERVO_RANGE = 30;
float RX_PID[3] = {0.0, 0.0, 0.0}; // p, i, d gains
float RY_PID[3] = {0.0, 0.0, 0.0};
float RZ_PID[3] = {0.0, 0.0, 0.0};
float AA_PID[3] = {0.0, 0.0, 0.0};
float AL_PID[3] = {0.0, 0.0, 0.0};
float TRX = 0; // target
float TRY = 0;
float TRZ = 0;
float TAL = 0;

// INTERNAL CONSTANTS
int SDA1_PIN = 21;
int SCL1_PIN = 22;
int SDA2_PIN = 23;
int SCL2_PIN = 19;
int EDF_PIN = 33;
int SERVO_PINS[4] = {14, 27, 25, 26};
int LED_PIN = 2;
int INPUT_PIN = 0;
int CALIBRATAE_ITERATIONS = 100;
float FILTER_STABLE_THRESHOLD = 0.3;
float FILTER_STABLE_MAX = 1000;
float WRAP_THRESHOLD = 300;
int FREQUENCY_UPDATE = 100;
int INTERVAL_UPDATE = 1000000 / FREQUENCY_UPDATE; // µs
float MAGNETOMETER_CALIBRATION_MATRIX[3][3] = {
  {0.016635, -0.001051, 0.000522},
  {-0.001051, 0.014854, 0.001723},
  {0.000522, 0.001723, 0.015777}
};
float MAGNETOMETER_OFFSET[3] = {80.914618, -68.543374, 19.054808};
float ACCELEROMETER_CALIBRATION_SCALE[3] = {1, 1, 1};
float GYROSCOPE_CALIBRATION_OFFSET[3] = {0, 0, 0};
float INIT_ANGLE[3] = {0, 0, 0};
float MADGWICK_BETA = 0.3;
float INIT_PITCH = 0;
float INIT_ROLL = 0;
float INIT_YAW = 0;

std::string BLE_NAME = "ESP32-BLE";
std::string BLE_INIT_VALUE = "hello";