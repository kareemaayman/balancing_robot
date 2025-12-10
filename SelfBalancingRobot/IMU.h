#ifndef IMU_H
#define IMU_H

#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>

class IMU {
private:
    MPU6050 mpu;
    bool dmpReady;
    uint8_t mpuIntStatus;
    uint16_t packetSize;
    uint16_t fifoCount;
    uint8_t fifoBuffer[64];
    
    Quaternion q;
    VectorFloat gravity;
    float ypr[3];
    float pitchAngle;
    
    volatile bool mpuInterrupt;
    static IMU* instancePtr;  // For static ISR
    
    static void dmpDataReadyISR();
    void dmpDataReady();
    
public:
    IMU();
    
    // Initialization
    bool initialize();
    bool isDMPReady() const { return dmpReady; }
    
    // Data reading
    bool updateAngle();
    float getPitchAngle() const { return pitchAngle; }
    float getYaw() const { return ypr[0] * 180.0 / M_PI; }
    float getRoll() const { return ypr[2] * 180.0 / M_PI; }
    
    // Raw quaternion access
    Quaternion getQuaternion() const { return q; }
    
    // Debug
    void printDiagnostics();
};

#endif
