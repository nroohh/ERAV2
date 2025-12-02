#ifndef CONFIG_H
#define CONFIG_H
#include <string>

// UUIDs
extern std::string SERVICE_UUID; 
extern std::string COMMAND_CHARACTERISTIC_UUID;
extern std::string TARGET_CHARACTERISTIC_UUID;
extern std::string SETTINGS_CHARACTERISTIC_UUID;

// EXTERNAL SETTINGS
extern float SERVO_OFFSETS[4];
extern float Z2XY_WEIGHT;
extern float SERVO_RANGE;
extern float RX_PID[3]; // p, i, d
extern float RY_PID[3];
extern float RZ_PID[3];
extern float AA_PID[3];
extern float AL_PID[3];
extern float TRX;
extern float TRY;
extern float TRZ;
extern float TAL;

// INTERNAL CONSTANTS
extern int SDA1_PIN;
extern int SCL1_PIN;
extern int SDA2_PIN;
extern int SCL2_PIN;
extern int EDF_PIN;
extern int SERVO_PINS[4];
extern int LED_PIN;
extern int INPUT_PIN;
extern int CALIBRATAE_ITERATIONS;
extern float FILTER_STABLE_THRESHOLD;
extern float FILTER_STABLE_MAX;
extern float WRAP_THRESHOLD;
extern int FREQUENCY_UPDATE;
extern int INTERVAL_UPDATE;
extern float MAGNETOMETER_CALIBRATION_MATRIX[3][3];
extern float MAGNETOMETER_OFFSET[3];
extern float ACCELEROMETER_CALIBRATION_SCALE[3];
extern float GYROSCOPE_CALIBRATION_OFFSET[3];
extern float INIT_ANGLE[3];
extern float MADGWICK_BETA;
extern float INIT_PITCH;
extern float INIT_ROLL;
extern float INIT_YAW;

extern std::string BLE_NAME;
extern std::string BLE_INIT_VALUE;

#endif
