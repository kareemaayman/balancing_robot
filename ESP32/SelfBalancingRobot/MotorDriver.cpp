#include "MotorDriver.h"

MotorDriver::MotorDriver(uint8_t rightIN1, uint8_t rightIN2, uint8_t rightENA,
                        uint8_t leftIN3, uint8_t leftIN4, uint8_t leftENB,
                        uint32_t pwmFreq, uint8_t pwmRes)
    : rightIN1(rightIN1), rightIN2(rightIN2), rightENA(rightENA),
      leftIN3(leftIN3), leftIN4(leftIN4), leftENB(leftENB),
      pwmFreq(pwmFreq), pwmRes(pwmRes),
      correctionLeft(Config::Motor::CORRECTION_LEFT_DEFAULT),
      correctionRight(Config::Motor::CORRECTION_RIGHT_DEFAULT) {}

void MotorDriver::initialize() {
    // Setup direction pins
    pinMode(rightIN1, OUTPUT);
    pinMode(rightIN2, OUTPUT);
    pinMode(leftIN3, OUTPUT);
    pinMode(leftIN4, OUTPUT);
    
    // Setup PWM (ESP32 LEDC)
    ledcAttach(rightENA, pwmFreq, pwmRes);
    ledcAttach(leftENB, pwmFreq, pwmRes);
    
    Serial.println("Motor driver initialized!");
}

void MotorDriver::move(int leftSpeed, int rightSpeed) {
    // Apply correction factors
    if (rightSpeed >= Config::MotorCorrection::CORRECTION_BAND_2_LOW && 
        rightSpeed < Config::MotorCorrection::CORRECTION_BAND_2_HIGH) {
      correctionRight = Config::MotorCorrection::CORRECTION_BAND_2_FACTOR;
    }
    else if (rightSpeed > Config::MotorCorrection::CORRECTION_BAND_1_LOW && 
             rightSpeed < Config::MotorCorrection::CORRECTION_BAND_1_HIGH) {
      correctionRight = Config::MotorCorrection::CORRECTION_BAND_1_FACTOR;
    }
    else if (rightSpeed < Config::MotorCorrection::CORRECTION_BAND_3_HIGH) {
      correctionRight = Config::MotorCorrection::CORRECTION_BAND_3_FACTOR;
    }
    leftSpeed = (int)(leftSpeed * correctionLeft);
    rightSpeed = (int)(rightSpeed * correctionRight);
    
    // LEFT MOTOR
    if (leftSpeed >= 0) {
        digitalWrite(leftIN3, HIGH);
        digitalWrite(leftIN4, LOW);
    } else {
        digitalWrite(leftIN3, LOW);
        digitalWrite(leftIN4, HIGH);
        leftSpeed = -leftSpeed;
    }
    
    // RIGHT MOTOR
    if (rightSpeed >= 0) {
        digitalWrite(rightIN1, HIGH);
        digitalWrite(rightIN2, LOW);
    } else {
        digitalWrite(rightIN1, LOW);
        digitalWrite(rightIN2, HIGH);
        rightSpeed = -rightSpeed;
    }
    
    // Apply speed constraints
    leftSpeed = constrain(leftSpeed, 0, Config::Motor::MAX_SPEED);
    rightSpeed = constrain(rightSpeed, 0, Config::Motor::MAX_SPEED);
    
    // Apply minimum speed threshold
    if (leftSpeed > 0 && leftSpeed < Config::Motor::MIN_SPEED) leftSpeed = Config::Motor::MIN_SPEED;
    if (rightSpeed > 0 && rightSpeed < Config::Motor::MIN_SPEED) rightSpeed = Config::Motor::MIN_SPEED;
    
    // Write PWM
    ledcWrite(leftENB, leftSpeed);
    ledcWrite(rightENA, rightSpeed);
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
