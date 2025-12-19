#include "MotorDriver.h"

MotorDriver::MotorDriver(uint8_t leftIN1, uint8_t leftIN2, uint8_t leftENA,
                         uint8_t rightIN3, uint8_t rightIN4, uint8_t rightENB,
                         uint32_t pwmFreq, uint8_t pwmRes)
    : leftIN1(leftIN1), leftIN2(leftIN2), leftENA(leftENA),
      rightIN3(rightIN3), rightIN4(rightIN4), rightENB(rightENB),
      pwmFreq(pwmFreq), pwmRes(pwmRes),
      correctionLeft(1.0), correctionRight(1.0) {}

void MotorDriver::initialize() {
    // Setup direction pins
    pinMode(leftIN1, OUTPUT);
    pinMode(leftIN2, OUTPUT);
    pinMode(rightIN3, OUTPUT);
    pinMode(rightIN4, OUTPUT);
    
    // Setup PWM (ESP32 LEDC)
    ledcAttach(leftENA, pwmFreq, pwmRes);
    ledcAttach(rightENB, pwmFreq, pwmRes);
    
    Serial.println("Motor driver initialized!");
}

void MotorDriver::move(int leftSpeed, int rightSpeed) {
    // Apply correction factors
    if (leftSpeed>=100&&leftSpeed<150){
      correctionLeft = 0.9;
    }
    else if (leftSpeed>80 && leftSpeed<100){
      correctionLeft = 0.84;
    }
    else if (leftSpeed < 80){
      correctionLeft = 0.8;
    }
    leftSpeed = (int)(leftSpeed * correctionLeft);
    rightSpeed = (int)(rightSpeed * correctionRight);
    
    // LEFT MOTOR
    if (leftSpeed >= 0) {
        digitalWrite(leftIN1, HIGH);
        digitalWrite(leftIN2, LOW);
    } else {
        digitalWrite(leftIN1, LOW);
        digitalWrite(leftIN2, HIGH);
        leftSpeed = -leftSpeed;
    }
    
    // RIGHT MOTOR
    if (rightSpeed >= 0) {
        digitalWrite(rightIN3, HIGH);
        digitalWrite(rightIN4, LOW);
    } else {
        digitalWrite(rightIN3, LOW);
        digitalWrite(rightIN4, HIGH);
        rightSpeed = -rightSpeed;
    }
    
    // Apply speed constraints
    leftSpeed = constrain(leftSpeed, 0, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);
    
    // Apply minimum speed threshold
    if (leftSpeed > 0 && leftSpeed < MIN_SPEED) leftSpeed = MIN_SPEED;
    if (rightSpeed > 0 && rightSpeed < MIN_SPEED) rightSpeed = MIN_SPEED;
    
    // Write PWM
    ledcWrite(leftENA, leftSpeed);
    ledcWrite(rightENB, rightSpeed);
}

void MotorDriver::stop() {
    move(0, 0);
}

void MotorDriver::setCorrection(double left, double right) {
    correctionLeft = left;
    correctionRight = right;
}

void MotorDriver::printDebug(int leftSpeed, int rightSpeed) {
    Serial.print("Left PWM: ");
    Serial.print(leftSpeed);
    Serial.print(" | Right PWM: ");
    Serial.println(rightSpeed);
}
