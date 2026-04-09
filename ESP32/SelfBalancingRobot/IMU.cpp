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
    mpu.setXAccelOffset(-124);
    mpu.setYAccelOffset(-196);
    mpu.setZAccelOffset(3392);
    mpu.setXGyroOffset(98);
    mpu.setYGyroOffset(-29);
    mpu.setZGyroOffset(0);
    // Try to initialize DMP
    int devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
        // turn on the DMP, now we'll get yaw from FIFO
        mpu.setDMPEnabled(true);
        mpu.resetFIFO();  
        packetSize = mpu.dmpGetFIFOPacketSize();
        Serial.print("DMP packetSize = ");
        Serial.println(packetSize);
        dmpReady = (packetSize > 0);
        //Serial.println("DMP initialized and enabled.");
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
    // Time step (seconds) for gyro rate (only used in fallback)
    unsigned long now = micros();
    dt = (now - lastYawTime) / 1000000.0f;
    lastYawTime = now;
    if (dt <= 0.0f) {
        return false;
    }

    if (!dmpReady || packetSize == 0) {
        // fallback or just return false
        return false;
    }

    // ----- DMP path -----
    uint16_t fifoCount = mpu.getFIFOCount();


    // Optional: handle overflow (very large fifoCount)
    if (fifoCount >= 1024) {
        mpu.resetFIFO();
        Serial.println("FIFO overflow");
        return false;
    }

    if (fifoCount < packetSize) {
        // no full packet yet -> no new yaw
        return false;
    }

    // Drain FIFO, keep the last (newest) packet
    while (fifoCount >= packetSize) {
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
    }

    Quaternion q;
    VectorFloat gravity;
    float ypr[3];

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    float yaw_deg = ypr[0] * 180.0f / PI;
    while (yaw_deg > 180.0f) yaw_deg -= 360.0f;
    while (yaw_deg < -180.0f) yaw_deg += 360.0f;
    yawAngle = yaw_deg;

    //Optionally compute gyroYawRate too if you still need it:
    // int16_t ax, ay, az, gx, gy, gz;
    // mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    // gyroYawRate = (float)gz / 131.0f;

    return true;
}

void IMU::resetYawReference() {
    // call this when robot is steady and you want "now" = 0°
    yawOffset = yawAngle;
}

float IMU::getYawDegRelative() const {
    float yaw = yawAngle - yawOffset;
    while (yaw > 180.0f) yaw -= 360.0f;
    while (yaw < -180.0f) yaw += 360.0f;
    return yaw;
}

void IMU::printDiagnostics() {
    Serial.print("Yaw: ");
    Serial.print(yawAngle, 2);
    Serial.print("° | Rate: ");
    Serial.print(gyroYawRate, 1);
    Serial.println("°/s");
}
