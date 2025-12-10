#include "IMU.h"
#include <Arduino.h>

// Static pointer for ISR
IMU* IMU::instancePtr = nullptr;

void IMU::dmpDataReadyISR() {
    if (IMU::instancePtr) {
        IMU::instancePtr->dmpDataReady();
    }
}

void IMU::dmpDataReady() {
    mpuInterrupt = true;
}

IMU::IMU() 
    : dmpReady(false), mpuInterrupt(false), pitchAngle(0) {
    IMU::instancePtr = this;
    memset(fifoBuffer, 0, sizeof(fifoBuffer));
}

bool IMU::initialize() {
    Serial.println("Initializing MPU6050...");
    
    // I2C setup for ESP32
    Wire.begin(21, 22, 400000);
    
    mpu.initialize();
    
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        return false;
    }
    
    uint8_t devStatus = mpu.dmpInitialize();
    
    // Set gyro offsets (calibrated values - adjust for your chip)
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setZAccelOffset(0);
    
    if (devStatus != 0) {
        Serial.print("DMP Initialization failed (code: ");
        Serial.print(devStatus);
        Serial.println(")");
        return false;
    }
    
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(15), dmpDataReadyISR, RISING);  // Pin 15 = MPU INT
    
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
    
    Serial.println("MPU6050 initialized successfully!");
    return true;
}

bool IMU::updateAngle() {
    if (!dmpReady) return false;
    
    fifoCount = mpu.getFIFOCount();
    
    // Check for overflow
    if ((mpu.getIntStatus() & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println("FIFO overflow!");
        return false;
    }
    
    // Check for data ready
    if (!(mpu.getIntStatus() & 0x02)) {
        return false;
    }
    
    // Wait for complete packet
    while (fifoCount < packetSize) {
        fifoCount = mpu.getFIFOCount();
    }
    
    // Read packet
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    
    // Calculate angle
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    // Convert to degrees with cleaner range (0-180, where 90 = vertical)
    pitchAngle = ypr[1] * 180.0 / M_PI + 90.0;
    
    return true;
}

void IMU::printDiagnostics() {
    Serial.print("Pitch: ");
    Serial.print(pitchAngle, 2);
    Serial.print("° | Yaw: ");
    Serial.print(getYaw(), 2);
    Serial.print("° | Roll: ");
    Serial.println(getRoll(), 2);
}
