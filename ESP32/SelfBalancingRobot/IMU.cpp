#include "IMU.h"
#include <Arduino.h>
#include <math.h>

IMU::IMU()
    : dt(0.0f),
      yawAngle(0.0f),
      gyroYawRate(0.0f) {
}

bool IMU::initialize() {
    Serial.println("Initializing MPU6050...");

    Wire.begin(21, 22, 400000);
    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        return false;
    }

    // Optional: calibrated offsets (tune these if needed)
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);

    // Try to initialize DMP
    int devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
        // turn on the DMP, now we'll get yaw from FIFO
        mpu.setDMPEnabled(true);
        packetSize = mpu.dmpGetFIFOPacketSize();
        dmpReady = true;
        Serial.println("DMP initialized and enabled.");
    } else {
        dmpReady = false;
        Serial.print("DMP initialization failed, code: ");
        Serial.println(devStatus);
        Serial.println("Falling back to raw gyro integration for yaw.");
    }

    lastYawTime = micros();

    Serial.println("MPU6050 ready (Yaw tracking).");
    return true;
}

bool IMU::updateYaw() {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    // Time step (seconds)
    unsigned long now = micros();
    dt = (now - lastYawTime) / 1000000.0f;
    lastYawTime = now;

    if (dt <= 0.0f) {
        return false;
    }

    // Gyro YAW rate from Z-axis (deg/s)
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gyroYawRate = (float)gz / 131.0f;

    if (dmpReady) {
        // If using DMP, read FIFO packet and extract yaw
        uint16_t fifoCount = mpu.getFIFOCount();
        if (fifoCount >= packetSize) {
            mpu.getFIFOBytes(fifoBuffer, packetSize);

            Quaternion q;
            VectorFloat gravity;
            float ypr[3];

            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            // ypr[0] is yaw in radians
            float yaw_deg = ypr[0] * 180.0f / PI;

            // Normalize yaw
            while (yaw_deg > 180.0f) yaw_deg -= 360.0f;
            while (yaw_deg < -180.0f) yaw_deg += 360.0f;

            yawAngle = yaw_deg;
        }
    } else {
        // Fallback: integrate raw gyro Z
        yawAngle += gyroYawRate * dt;
        
        // Normalize yaw to -180 to 180 degrees
        while (yawAngle > 180.0f) yawAngle -= 360.0f;
        while (yawAngle < -180.0f) yawAngle += 360.0f;
    }

    return true;
}

void IMU::printDiagnostics() {
    Serial.print("Yaw: ");
    Serial.print(yawAngle, 2);
    Serial.print("° | Rate: ");
    Serial.print(gyroYawRate, 1);
    Serial.println("°/s");
}
