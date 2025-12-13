#include "IMU.h"
#include <Arduino.h>
#include <math.h>

/*
  IMU.cpp
  --------
  Kalman pitch estimator
  Pitch definition matches:
  pitchAcc = atan2(-ax, sqrt(ay^2 + az^2))
  Gyro source = gy
  Timing = micros()
*/

IMU::IMU()
    : pitchAngle(0.0f),
      gyroPitchRate(0.0f),
      lastTime(0),
      accPitch(0.0f),
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
    kalmanPitch.setQangle(0.001f);
    kalmanPitch.setQbias(0.003f);
    kalmanPitch.setRmeasure(0.03f);

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

    // Accelerometer pitch (EXACT same formula)
    accPitch = atan2(-ax, sqrt((float)ay * ay + (float)az * az)) * 180.0f / PI;

    // Gyro pitch rate from Y-axis (deg/s)
    gyroPitchRate = (float)gy / 131.0f;

    // Kalman filter
    pitchAngle = kalmanPitch.getAngle(accPitch, gyroPitchRate, dt);

    return true;
}

void IMU::printDiagnostics() {
    Serial.print("AccPitch: ");
    Serial.print(accPitch, 2);
    Serial.print(" | GyroY: ");
    Serial.print(gyroPitchRate, 2);
    Serial.print(" | KalmanPitch: ");
    Serial.print(pitchAngle, 2);
    Serial.print(" | dt(us): ");
    Serial.println(dt * 1e6, 0);
}
