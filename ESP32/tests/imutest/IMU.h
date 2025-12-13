#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include <MPU6050.h>
#include <Kalman.h>

class IMU {
private:
    MPU6050 mpu;
    Kalman kalmanPitch;

	float pitchAngle = 0.0f;
	float gyroPitchRate = 0.0f;
	float accPitch = 0.0f;
	float dt = 0.0f;
	unsigned long lastTime = 0;


public:
    IMU();

    bool initialize();
    bool updateAngle();

    float getPitchAngle() const { return pitchAngle; }
    void printDiagnostics();
};


#endif
