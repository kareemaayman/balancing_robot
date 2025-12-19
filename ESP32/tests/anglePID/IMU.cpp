#include "IMU.h"
#include <Arduino.h>
#include <math.h>

IMU::IMU()
    : rollAngle(0.0f),
      gyroRollRate(0.0f),
      lastTime(0),
      accRoll(0.0f),
      dt(0.0f) {
}

bool IMU::initialize() {
    Serial.println("Initializing MPU6050 (Kalman pitch)...");

    Wire.begin(21, 22, 400000);
    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        return false;
    }

    // Optional: calibrated offsets
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);

    // Kalman parameters (same behavior as your test)
    kalmanRoll.setQangle(0.001f);
    kalmanRoll.setQbias(0.003f);
    kalmanRoll.setRmeasure(0.03f);

    lastTime = micros();

    Serial.println("MPU6050 ready (Kalman pitch).");
    return true;
}

bool IMU::updateAngle() {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Time step (seconds)
    unsigned long now = micros();
    dt = (now - lastTime) / 1000000.0f;
    lastTime = now;

    if (dt <= 0.0f) {
        return false;
    }

    // Accelerometer ROLL
    accRoll = atan2(-ay, az) * 180.0f / PI;

    // Gyro ROLL rate from X-axis (deg/s)
    gyroRollRate = (float)gx / 131.0f;  // deg/s

    // Kalman filter
    kalmanRollAngle = kalmanRoll.getAngle(accRoll, gyroRollRate, dt);
    rollAngle = kalmanRollAngle + 90; // offset may be 0–90 depending on mounting

    return true;
}

void IMU::printDiagnostics() {
    Serial.print(" Roll: ");
    Serial.println(rollAngle, 2);
}
