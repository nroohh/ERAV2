// #include <Servo.h>
// #include <MPU6050.h>
// #include <math.h>
// #include <NeoSWSerial.h>
// #include "TinyGPS++.h"

using namespace std;

class Axis {
  public:
    double c; // current
    double pg; // p gain
    double ig; // i gain
    double dg; // d gain
    double tgt; // target
    double pre = 0; // previous error
    double output;
    double integral = 0; // accumilated error
    double derivative = 0; // change in  error

    Axis(double c, double pg, double ig, double dg, double tgt) : c(c), pg(pg), ig(ig), dg(dg), tgt(tgt) {}; // current, proportionial gain, integral gain, derivative gain, target, output

    void pid(double dt) {
      output = 0;

      double error = tgt - c;

      // P
      double p = pg * error; 
      // I
      integral += error * dt;
      double i = ig * integral;
      // D
      derivative = (error - pre) / dt;
      double d = dg * derivative;
      pre = error;

      output = p + i + d;

      if (abs(output) > 1) { // limits output to values between -1 and 1
          output = output / abs(output);
      }
    }
};

struct State {
  double input;
  double value;
  double variance;
};


// INIT VAR
MPU6050 mpu;
Servo SERVO_A;
Servo SERVO_B;
Servo SERVO_C;
Servo SERVO_D;
Servo EDF;
TinyGPSPlus gps;
NeoSWSerial TS100(2, 3);

Axis rotationX(0, 0.07, 0, 0.01, 0); // current, p, i, d, target
Axis rotationY(0, 0.07, 0, 0.01, 0);
Axis rotationZ(0, 0.1, 0, 0, 0);
Axis positionZ(0, 0.5, 0.01, 0.01, 0.3);

const int servoPins[] = {7, 6, 5, 4}; // A, B, C, D
const int edfPin = 9;
const double pi = 3.141592653589793;
const double g = -9.81;
const double gU = 131.0;
const double aU = 16384.0;
const double w = 0.8; // weight on rotationZ | rotationX & rotationY
const double servoU = 45;// maximum angle of servo rotation(in degrees)
const double _throttle = 1500; // medium throttle(microseconds)
const double throttleU = 500; // range of throttle(microseconds)
double accel[3] = {0, 0, 0}; // {x, y, z}
double angle[3] = {0, 0, 0}; // {x, y, z}
double gyro[3] = {0, 0, 0};
State _acceleration = {0, 0, 0}; // previous
State _velocity = {0, 0, 0}; 
State _position = {0, 0, 0}; 
State acceleration = {0, 0, 0}; // up direction
State velocity = {0, 0, 0}; // up direction
State position = {0, 0, 0};  // up direction
State dt = {0, 0, 0.001};
State thetaXK = {0, 0, 1}; // kalman {initial value, initial uncertainty}
State omegaXM = {0, 0, 0.01}; // measured
State thetaXM  = {0, 0, 0.01};
State thetaYK = {0, 0, 1};
State omegaYM = {0, 0, 0.01};
State thetaYM = {0, 0, 0.01};
State altitudeK = {0, 0, 1};
State climbM = {0, 0, 0.05}; // vertical velocity
State altitudeM  = {0, 0, 10};
unsigned long ct = 0; // current time
unsigned long pt = 0; // previous time

// calibration variables(bias);
double angleB[3] = {0, 0, 0}; // {x, y, z}
double gyroB[3] = {0, 0, 0};
double accelerationB = 0; // up direction
double altitudeB = 0; // initial


// FUNCTIONS
bool isServoAttached(int pin) {
  int pwmValue = pulseIn(pin, HIGH);
  if (pwmValue > 1000 && pwmValue < 2000) { // Typical range for servos
    return true;
  } else {
    return false; 
  }
}

State kalman(State p, State dp, State dt, State m) {  // previous, delta previous, delta time, measured
    // Prediction step
    State predicted;
    predicted.value = p.value + dp.value * dt.value;
    predicted.variance = p.variance + dt.variance * dp.variance;

    // Compute Kalman Gain
    double gain = predicted.variance / (predicted.variance + m.variance);

    // Update step
    State filtered;
    filtered.input = 0;
    filtered.value = predicted.value + gain * (m.value - predicted.value);
    filtered.variance = (1 - gain) * predicted.variance;

    return filtered;
}

double accelDown() {
  
  // return -g * ((-accel[0] * sin(thetaYK.value * (PI / 180.0)) + accel[1] * cos(thetaYK.value * (PI / 180.0)) * sin(thetaXK.value * (PI / 180.0)) + accel[2] * cos(thetaYK.value * (PI / 180.0)) * cos(thetaXK.value * (PI / 180.0))) - 1) + accelerationB; // equation from rotational matrix[ms^-2]
  return -accel[2] * g + g;
  // return  -accel[2] * g - g*cos(thetaXK.value * (PI / 180.0)) *cos(thetaYK.value * (PI / 180.0));
}

double highPass(double alpha, double pv, double pvi, double c) {
  double compute = alpha * (pv + c - pvi);
  return compute;
}

double lowPass(double beta, double pv, double c, double dl = 0) {
    double compute =  (1 - beta) * pv + beta * c;
    return compute;
}

double limit(double l, double pv, double cv) {
  if (abs(cv-pv) < l) {
    return pv;
  } else {
    return cv;
  }
} 


void compute(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz) {
  accel[0] = ax / aU;
  accel[1] = ay / aU;
  accel[2] = az / aU;

  gyro[0] = gx / gU + gyroB[0];
  gyro[1] = gy / gU + gyroB[1];
  gyro[2] = gz / gU + gyroB[2];

  angle[0] = atan2(accel[1], sqrt(accel[0] * accel[0] + accel[2] * accel[2])) * (180.0 / PI) + angleB[0];
  angle[1] = atan2(-accel[0],sqrt(accel[1] * accel[1] + accel[2] * accel[2])) * (180.0 / PI) + angleB[1];
  angle[2] += gyro[2] * dt.value;

  // KALMAN FILTER
  omegaXM.value = gyro[0];
  thetaXM.value = angle[0];
  thetaXK = kalman(thetaXK, omegaXM, dt, thetaXM);
  omegaYM.value = gyro[1];
  thetaYM.value = angle[1];
  thetaYK = kalman(thetaYK, omegaYM, dt, thetaYM);

  acceleration.input = accelDown() + accelerationB;
  acceleration.value = lowPass(0.1, _acceleration.value, acceleration.input);

  velocity.input += 0.5 * (acceleration.value + _acceleration.value) * dt.value;
  velocity.value = highPass(0.9, _velocity.value, _velocity.input, velocity.input);
  
  climbM.value = velocity.value;
  if (gps.altitude.isUpdated()) {
    altitudeM.value = gps.altitude.meters() + altitudeB;
  }
  altitudeK = kalman(altitudeK, climbM, dt, altitudeM);

  _position = position;
  _velocity = velocity;
  _acceleration = acceleration;
}

void initialize() {
  Serial.println("\n");
  Serial.println("initializing...");
  Serial.begin(9600);

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050[X]");
    while (1);
  } else {
    Serial.println("MPU6050[V]");
  }
  
  SERVO_A.attach(servoPins[0]);
  SERVO_B.attach(servoPins[1]);
  SERVO_C.attach(servoPins[2]);
  SERVO_D.attach(servoPins[3]);

  if (isServoAttached(servoPins[0])) {
    Serial.println("SERVO_A[V]");
  } else {
    Serial.println("SERVO_A[X]");
  }
  if (isServoAttached(servoPins[1])) {
    Serial.println("SERVO_B[V]");
  } else {
    Serial.println("SERVO_B[X]");
  }
  if (isServoAttached(servoPins[2])) {
    Serial.println("SERVO_C[V]");
  } else {
    Serial.println("SERVO_C[X]");
  }
  if (isServoAttached(servoPins[3])) {
    Serial.println("SERVO_D[V]");
  } else {
    Serial.println("SERVO_D[X]");
  }

  TS100.begin(9600);
  unsigned long time = millis();
  double await = 60000;

  while (true) {
    if (TS100.available()) {
      Serial.println("TS100[V]");
      break;
    } else if (millis() >= (time + await)) {
      Serial.println("TS100[X]");
      break;
    }

    delay(500);
  }


  EDF.attach(edfPin);
  EDF.writeMicroseconds(1000); // Initialize the ESC (minimum throttle)
  delay(2000); // Wait for 2 seconds for ESC to initialize
  Serial.println("EDF[V]");
}

void calibrate() {
  // to calibrate, leave the model in a flat surface for the duration specified.
  unsigned long t = millis();
  int count = 0;
  double duration = 5000; // milliseconds
  double _angleB[3] = {0, 0, 0}; // {x, y, z}
  double _gyroB[3] = {0, 0, 0};  
  double _accelerationB = 0;  
  double _velocityB = 0;  
  double _positionB = 0;  
  double _altitudeB = 0;

  Serial.println("calibrating...");

  
  // calibrate
  while (millis() - t < duration) {
    // DELTA TIME
    ct = millis();  // Get current time in milliseconds
    dt.value = (ct - pt) / 1000.0; // Convert ms to seconds
    pt = ct; // Update previous time
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    while (TS100.available()) {
      byte data = TS100.read();
      gps.encode(data);
    } // read GPS
    if (gps.altitude.isUpdated() && (gps.altitude.meters() != 0)) {
      _altitudeB += gps.altitude.meters();
      mpu.getAcceleration(&ax, &ay, &az);
      mpu.getRotation(&gx, &gy, &gz);

      compute(ax, ay, az, gx, gy, gz);

      _angleB[0] += angle[0];
      _angleB[1] += angle[1];
      _angleB[2] += angle[2];

      _gyroB[0] += gyro[0];
      _gyroB[1] += gyro[1];
      _gyroB[2] += gyro[2];

      _accelerationB += acceleration.input;

      count++;
    }
  }

  angleB[0] = -_angleB[0] / count;
  angleB[1] = -_angleB[1] / count;
  angleB[2] = -_angleB[2] / count;
  gyroB[0] = -_gyroB[0] / count;
  gyroB[1] = -_gyroB[1] / count;
  gyroB[2] = -_gyroB[2] / count;
  accelerationB = -_accelerationB / count;
  altitudeB = -_altitudeB / count;

  acceleration = {0, 0, 0};
  velocity = {0, 0, 0};
  position = {0, 0, 0};
  _acceleration = {0, 0, 0};
  _velocity = {0, 0, 0};
  _position = {0, 0, 0};

  delay(1000);
}

void setup() {
  initialize();
  calibrate();
}

void loop() {
  // DELTA TIME
  ct = millis();  // Get current time in milliseconds
  dt.value = (ct - pt) / 1000.0; // Convert ms to seconds
  pt = ct; // Update previous time

  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  while (TS100.available()) {
    byte data = TS100.read();
    gps.encode(data);
  } // read GPS

  compute(ax, ay, az, gx, gy, gz);

  // // SET VALUE
  rotationX.c = thetaXK.value;
  rotationY.c = thetaYK.value;
  rotationZ.c = gyro[2];
  positionZ.c = altitudeK.value;
  Serial.print("climb: "); Serial.print(velocity.value);
  Serial.print(", measured:"); Serial.print(altitudeM.value);
  Serial.print(", kalman:"); Serial.print(altitudeK.value);
  Serial.print(", bias:"); Serial.println(altitudeB);




  // PID
  rotationX.pid(dt.value);
  rotationY.pid(dt.value);
  rotationZ.pid(dt.value);
  positionZ.pid(dt.value);

  

  // ACTION
  SERVO_A.write(90 + ((w * rotationX.output) - ((1 - w) * rotationZ.output)) * servoU);
  SERVO_C.write(90 + ((-1 * w * rotationX.output) - ((1 - w) * rotationZ.output)) * servoU);
  SERVO_B.write(90 + ((w * rotationY.output) - ((1 - w) * rotationZ.output)) * servoU);
  SERVO_D.write(90 + ((-1 * w * rotationY.output) - ((1 - w) * rotationZ.output)) * servoU);
  // EDF.writeMicroseconds(_throttle + positionZ.output * throttleU);

  delay(50);
}
