#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include "Config.h"
#include <MPU6050_6Axis_MotionApps20.h>
#include <MPU6050.h>

class IMU {
private:
    MPU6050 mpu;
    float dt;
    unsigned long lastYawTime = 0;
    
    // Yaw tracking only
    float yawAngle = 0.0f;
    float gyroYawRate;
    
    // DMP (MPU6050) support
    bool dmpReady = false;
    uint16_t packetSize = 0;
    uint8_t fifoBuffer[64];

public:
    IMU();

    bool initialize();
    bool updateYaw();
    
    float getYawAngle() const { return yawAngle; }
    float getGyroYawRate() const { return gyroYawRate; }
    void resetYaw() { yawAngle = 0.0f; }
    void printDiagnostics();
};

#endif
