
#include <Adafruit_AHRS_Madgwick.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <MPU9250_asukiaaa.h>
#define SDA_PIN 21
#define SCL_PIN 22


Preferences prefs;
MPU9250_asukiaaa mpu;
Adafruit_Madgwick filter;
Servo servos[4];
int servoPins[] = {14, 27, 26, 25};


// CALIBRATION VAR
float accelC[] = {0, 0, 0}; // accelerometer calibration scale
float gyroC[] = {0, 0, 0}; // gyroscope calibration offset
float magnC[] = {0, 0, 0}; // magnetometer calibration bias

const int fUpdate = 10; // frequency update [Hz]
const int iUpdate = 1000 / fUpdate; // milliseconds (10 ms) interval update
unsigned long tStart; // start dt
unsigned long dt = 100UL;


void waitKey() {
  while (!Serial.available()) {
  }
  while (Serial.available()) {
    Serial.read();
  }
}

void calibrateAccelerometer() {
  // MPU9250 ACCELEROMETER CALIBRATTION
  int iteration = 50;

  Serial.println("MPU9250 accelerometer: ");
  Serial.println("set x axis pointing down[press any key to calibrate]...");

  // ACCEL X
  waitKey();
  float maxValue = -INFINITY;
  for (int i = 0; i < iteration; i++) {
    mpu.accelUpdate();
    float value = mpu.accelX();
    if (value > maxValue) maxValue = value;
  }
  Serial.println("set x axis pointing up[press any key to calibrate]...");
  waitKey();
  float minValue = INFINITY;
  for (int i = 0; i < iteration; i++) {
    mpu.accelUpdate();
    float value = mpu.accelX();
    if (value < minValue) minValue = value;
  }
  accelC[0] = 2 / (maxValue - minValue);

  // ACCEL Y
  Serial.println("set y axis pointing down[press any key to calibrate]...");
  waitKey();
  maxValue = -INFINITY;
  for (int i = 0; i < iteration; i++) {
    mpu.accelUpdate();
    float value = mpu.accelY();
    if (value > maxValue) maxValue = value;
  }
  Serial.println("set y axis pointing up[press any key to calibrate]...");
  waitKey();
  minValue = INFINITY;
  for (int i = 0; i < iteration; i++) {
    mpu.accelUpdate();
    float value = mpu.accelY();
    if (value < minValue) minValue = value;
  }
  accelC[1] = 2 / (maxValue - minValue);

  // ACCEL Z
  Serial.println("set z axis pointing down[press any key to calibrate]...");
  waitKey();
  maxValue = -INFINITY;
  for (int i = 0; i < iteration; i++) {
    mpu.accelUpdate();
    float value = mpu.accelZ();
    if (value > maxValue) maxValue = value;
  }
  Serial.println("set z axis pointing up[press any key to calibrate]...");
  waitKey();
  minValue = INFINITY;
  for (int i = 0; i < iteration; i++) {
    mpu.accelUpdate();
    float value = mpu.accelZ();
    if (value < minValue) minValue = value;
  }
  accelC[2] = 2 / (maxValue - minValue);

  Serial.println("accelerometer calibrated;");
}

void calibrateGyroscope() {
  // MPU9250 GYROSCOPE CALIBRATTION
  int iteration = 500;

  Serial.println("MPU9250 gyroscope: ");
  Serial.println("hold still...");

  waitKey();

  float sum[] = {0, 0, 0};
  for (int i = 0; i < iteration; i++) {
    mpu.gyroUpdate();
    sum[0] += mpu.gyroX();
    sum[1] += mpu.gyroY();
    sum[2] += mpu.gyroZ();

    delay(10);
  }

  gyroC[0] = sum[0] / iteration;
  gyroC[1] = sum[1] / iteration;
  gyroC[2] = sum[2] / iteration;

  Serial.println("gyroscope calibrated;");
}

void calibrateMagenetometer() {
  // MPU9250 MAGNETOMETER CALIBRATTION
  int iteration = 5000;
  float range[] = {INFINITY, -INFINITY, INFINITY, -INFINITY, INFINITY, -INFINITY}; // min max

  Serial.println("MPU9250 magnetometer: ");
  Serial.println("rotate in all orientations...");

  waitKey();

  float magnX;
  float magnY;
  float magnZ;

  for (int i = 0; i < iteration; i++) {
    mpu.magUpdate();
    magnX = mpu.magX();
    magnY = mpu.magY();
    magnZ = mpu.magZ();
    Serial.print(magnX);
    if (magnX < range[0]) range[0] = magnX;
    if (magnX > range[1]) range[1] = magnX;
    if (magnY < range[2]) range[2] = magnY;
    if (magnY > range[3]) range[3] = magnY;
    if (magnZ < range[4]) range[4] = magnZ;
    if (magnZ > range[5]) range[5] = magnZ;
    delay(10);
  }

  magnC[0] = (range[0] + range[1]) / 2;
  magnC[1] = (range[2] + range[3]) / 2;
  magnC[2] = (range[4] + range[5]) / 2;


  Serial.println("gyroscope calibrated;");
}


// void calibrateMagenetometer() {
//   // MPU9250 MAGNETOMETER CALIBRATTION
//   float range[] = {INFINITY, -INFINITY, INFINITY, -INFINITY, INFINITY, -INFINITY}; // min max

//   Serial.println("MPU9250 magnetometer: ");
//   Serial.println("rotate in all orientations...");

//   waitKey();

//   float magnX;
//   float magnY;
//   float magnZ;

//   while (!Serial.available()) {
//     mpu.magUpdate();
//     magnX = mpu.magX();
//     magnY = mpu.magY();
//     magnZ = mpu.magZ();
//     Serial.print("Raw: 0,0,0,0,0,0,"); Serial.print((int)magnX * 10); Serial.print(","); Serial.print((int)magnY * 10); Serial.print(","); Serial.println((int)magnZ * 10); 
//     delay(10);
//   }

//   Serial.println("gyroscope calibrated;");
// }


void calibrate() {
  // calibrateAccelerometer();
  // calibrateGyroscope();
  calibrateMagenetometer();
}

void initialize() {
  // SERVOS INIT
  for (int i = 0; i < 4; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(90);
  }
  Serial.println("SERVOS initialized;");

  // MPU9250 INIT
  Wire.begin(SDA_PIN, SCL_PIN);
  mpu.setWire(&Wire);
  mpu.beginAccel();
  mpu.beginGyro();
  mpu.beginMag();
  Serial.println("MPU9250 initialized;");

  // FILTER
  filter.begin(fUpdate);
}

void setup() {
  Serial.begin(115200);
  initialize();
  calibrate();
}

void loop() {
  tStart = millis();


  mpu.accelUpdate();
  mpu.gyroUpdate();
  mpu.magUpdate();

  // float ax = mpu.accelX() * accelC[0];
  // float ay = mpu.accelY() * accelC[1];
  // float az = mpu.accelZ() * accelC[2];

  float ax = mpu.accelX();
  float ay = mpu.accelY();
  float az = mpu.accelZ();

  float gx = mpu.gyroX() - gyroC[0];
  float gy = mpu.gyroY() - gyroC[1];
  float gz = mpu.gyroZ() - gyroC[2];

  float mx = mpu.magX() - magnC[0];
  float my = mpu.magY() - magnC[1];
  float mz = mpu.magZ() - magnC[2];

  filter.update(gx,gz,gy,ax,az,ay,mx,mz,my);


  Serial.print("Raw Magn: ");
  Serial.print(mx); Serial.print(", ");
  Serial.print(my); Serial.print(", ");
  Serial.print(mz);Serial.print("head ");
  Serial.print(atan2(my, mx));
  Serial.print("C: "); Serial.print(magnC[0]); Serial.print(", "); Serial.print(magnC[1]); Serial.print(", "); Serial.println(magnC[2]);

  dt = millis() - tStart;
  if (dt < iUpdate) {
    delay(iUpdate - dt);
  }
}
