#include <hnu.h>

HNU::HNU() {}

void HNU::init() {
    initMPU();
    initTOF();
    initFilter();
    vl.startRanging();
}

void HNU::calibrate() {
    // calibrateAccel();
    calibrateGyro();
    // calibrateMag();
    stabilizeFilter();
}

double wrap(double reference, double c) {
    if (reference > 0) {
        if (c < (- 180 + reference)) {
            return c + 360 - reference;
        } else {
            return c - reference;
        }
    } else {
        if (c > (180 + reference)) {
            return c - 360 - reference;
        } else {
            return c - reference;
        }
    }
}

void HNU::update() {
    
    distance = vl.distance(); // distance in millimeters
    mpu.accelUpdate();
    mpu.gyroUpdate();
    mpu.magUpdate();
    adjustAccel(mpu.accelX(),mpu.accelY(),mpu.accelZ());
    adjustGyro(mpu.gyroX(),mpu.gyroY(),mpu.gyroZ());
    adjustMagneto(mpu.magX(),mpu.magY(),mpu.magZ());
    filter.update(gyro[0], gyro[1], gyro[2], -accel[0], -accel[1], accel[2], magneto[0], magneto[1], magneto[2], dt); // microsecond to sec

    pitch = filter.getPitch();
    roll = wrap(INIT_ROLL, filter.getRoll());
    yaw = wrap(INIT_YAW, filter.getYaw() - 180);

    pRoll = roll;
    pPitch = pitch;
    pYaw = yaw;
}

void HNU::timerStart() {
    ts = micros();
}

void HNU::timerEnd(bool await = true) {
    if (!await) {
        te = micros() - ts;
        if (te < INTERVAL_UPDATE) {
            delay((INTERVAL_UPDATE - te)/1000);
        }
    } else {
        dt = (double) (micros() - ts) / 1000000.0f; // microsecond to sec
    }
}

// private methods

void HNU::initMPU() {
    Wire.begin(SDA1_PIN, SCL1_PIN);
    mpu.setWire(&Wire);
    mpu.beginAccel();
    mpu.beginGyro();
    mpu.beginMag();
}

void HNU::initTOF() {
    Wire1.begin(SDA2_PIN, SCL2_PIN);
    if (!vl.begin(0x0C, &Wire1)) {
        while (1);
    }
}

void HNU::initFilter() {
    filter.begin(FREQUENCY_UPDATE);
    filter.setBeta(MADGWICK_BETA);
}

void HNU::calibrateAccel() {
    blink(LED_PIN, 100, 3);

    // ACCEL X
    float maxValue = -INFINITY;
    for (int i = 0; i < CALIBRATAE_ITERATIONS; i++) {
        mpu.accelUpdate();
        float value = mpu.accelX();
        if (value > maxValue) maxValue = value;
    }
    hault(INPUT_PIN);
    float minValue = INFINITY;
    for (int i = 0; i < CALIBRATAE_ITERATIONS; i++) {
        mpu.accelUpdate();
        float value = mpu.accelX();
        if (value < minValue) minValue = value;
    }
    ACCELEROMETER_CALIBRATION_SCALE[0] = 2 / (maxValue - minValue);

    // ACCEL Y
    hault(INPUT_PIN);
    maxValue = -INFINITY;
    for (int i = 0; i < CALIBRATAE_ITERATIONS; i++) {
        mpu.accelUpdate();
        float value = mpu.accelY();
        if (value > maxValue) maxValue = value;
    }
    hault(INPUT_PIN);
    minValue = INFINITY;
    for (int i = 0; i < CALIBRATAE_ITERATIONS; i++) {
        mpu.accelUpdate();
        float value = mpu.accelY();
        if (value < minValue) minValue = value;
    }
    ACCELEROMETER_CALIBRATION_SCALE[1] = 2 / (maxValue - minValue);

    // ACCEL Z
    hault(INPUT_PIN);
    maxValue = -INFINITY;
    for (int i = 0; i < CALIBRATAE_ITERATIONS; i++) {
        mpu.accelUpdate();
        float value = mpu.accelZ();
        if (value > maxValue) maxValue = value;
    }
    hault(INPUT_PIN);
    minValue = INFINITY;
    for (int i = 0; i < CALIBRATAE_ITERATIONS; i++) {
        mpu.accelUpdate();
        float value = mpu.accelZ();
        if (value < minValue) minValue = value;
    }
    ACCELEROMETER_CALIBRATION_SCALE[2] = 2 / (maxValue - minValue);
}

void HNU::calibrateGyro() {
    blink(LED_PIN, 100, 3);

    float sum[] = {0, 0, 0};
    for (int i = 0; i < CALIBRATAE_ITERATIONS; i++) {
        mpu.gyroUpdate();
        sum[0] += mpu.gyroX();
        sum[1] += mpu.gyroY();
        sum[2] += mpu.gyroZ();

        delay(10);
    }

    GYROSCOPE_CALIBRATION_OFFSET[0] = sum[0] / CALIBRATAE_ITERATIONS;
    GYROSCOPE_CALIBRATION_OFFSET[1] = sum[1] / CALIBRATAE_ITERATIONS;
    GYROSCOPE_CALIBRATION_OFFSET[2] = sum[2] / CALIBRATAE_ITERATIONS;
}

void HNU::calibrateMag() {
    // magnetometer calibration has to be done manually using magneto1.2 software!
}

void HNU::adjustAccel(float ax, float ay, float az) {
    accel[0] = ax * ACCELEROMETER_CALIBRATION_SCALE[0]; 
    accel[1] = ay * ACCELEROMETER_CALIBRATION_SCALE[1]; 
    accel[2] = az * ACCELEROMETER_CALIBRATION_SCALE[2];
}

void HNU::adjustGyro(float gx, float gy, float gz) {
    gyro[0] = gx - GYROSCOPE_CALIBRATION_OFFSET[0]; 
    gyro[1] = gy - GYROSCOPE_CALIBRATION_OFFSET[1]; 
    gyro[2] = gz - GYROSCOPE_CALIBRATION_OFFSET[2];
}

void HNU::adjustMagneto(float mx,float my,float mz) {
    magneto[0] = MAGNETOMETER_CALIBRATION_MATRIX[0][0]*(mx - MAGNETOMETER_OFFSET[0]) + MAGNETOMETER_CALIBRATION_MATRIX[0][1]*(my - MAGNETOMETER_OFFSET[1]) + MAGNETOMETER_CALIBRATION_MATRIX[0][2]*(mz - MAGNETOMETER_OFFSET[2]);
    magneto[1] = MAGNETOMETER_CALIBRATION_MATRIX[1][0]*(mx - MAGNETOMETER_OFFSET[0]) + MAGNETOMETER_CALIBRATION_MATRIX[1][1]*(my - MAGNETOMETER_OFFSET[1]) + MAGNETOMETER_CALIBRATION_MATRIX[1][2]*(mz - MAGNETOMETER_OFFSET[2]);
    magneto[2] = MAGNETOMETER_CALIBRATION_MATRIX[2][0]*(mx - MAGNETOMETER_OFFSET[0]) + MAGNETOMETER_CALIBRATION_MATRIX[2][1]*(my - MAGNETOMETER_OFFSET[1]) + MAGNETOMETER_CALIBRATION_MATRIX[2][2]*(mz - MAGNETOMETER_OFFSET[2]);
}

bool isStable(float prev, float current, float threshold) {
    return fabs(current - prev) < threshold;
}

void HNU::stabilizeFilter() {
    int count = 0;
    while (count < FILTER_STABLE_MAX) {
        timerStart();

        update();

        if (isStable(pRoll, roll, FILTER_STABLE_THRESHOLD) &&
            isStable(pPitch, pitch, FILTER_STABLE_THRESHOLD) &&
            isStable(pYaw, yaw, FILTER_STABLE_THRESHOLD)) {
            count++;
        } else {
            count = 0;
        }

        timerEnd(true);
    }

    INIT_ROLL = 180; // predetermined due to sensor orientation
    INIT_PITCH = 0;
    INIT_YAW = yaw;
}