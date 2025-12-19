#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

class MotorDriver {
private:
    // Left motor pins
    const uint8_t leftIN1;
    const uint8_t leftIN2;
    const uint8_t leftENA;
    
    // Right motor pins
    const uint8_t rightIN3;
    const uint8_t rightIN4;
    const uint8_t rightENB;
    
    // PWM settings
    const uint32_t pwmFreq;
    const uint8_t pwmRes;
    
    // Correction factors
    double correctionLeft;
    double correctionRight;
    
    // Constants
    static const uint8_t MIN_SPEED = 60;
    static const uint8_t MAX_SPEED = 255;
    
public:
    MotorDriver(uint8_t leftIN1, uint8_t leftIN2, uint8_t leftENA,
                uint8_t rightIN3, uint8_t rightIN4, uint8_t rightENB,
                uint32_t pwmFreq = 1000, uint8_t pwmRes = 8);
    
    void initialize();
    
    // Motor control (-255 to +255)
    void move(int leftSpeed, int rightSpeed);
    void stop();
    
    // Correction factors for drift compensation
    void setCorrection(double left, double right);
    void printDebug(int leftSpeed, int rightSpeed);
};

#endif
