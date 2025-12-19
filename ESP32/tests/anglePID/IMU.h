#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include <MPU6050.h>
#include <Kalman.h>

class IMU {
private:
    MPU6050 mpu;
    Kalman kalmanRoll;
    float kalmanRollAngle;
    float rollAngle;
    float gyroRollRate;
    float accRoll;
	float dt;
	unsigned long lastTime = 0;


public:
    IMU();

    bool initialize();
    bool updateAngle();

    float getRollAngle() const { return rollAngle; }
    void printDiagnostics();
};


#endif
