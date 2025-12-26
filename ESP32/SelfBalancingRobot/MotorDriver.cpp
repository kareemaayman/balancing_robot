#include "MotorDriver.h"

MotorDriver::MotorDriver(uint8_t leftIN1, uint8_t leftIN2, uint8_t leftENA,
                         uint8_t rightIN3, uint8_t rightIN4, uint8_t rightENB,
                         uint32_t pwmFreq, uint8_t pwmRes)
    : leftIN1(leftIN1), leftIN2(leftIN2), leftENA(leftENA),
      rightIN3(rightIN3), rightIN4(rightIN4), rightENB(rightENB),
      pwmFreq(pwmFreq), pwmRes(pwmRes),
      correctionLeft(Config::Motor::CORRECTION_LEFT_DEFAULT),
      correctionRight(Config::Motor::CORRECTION_RIGHT_DEFAULT) {}

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
    if (leftSpeed >= Config::MotorCorrection::CORRECTION_BAND_2_LOW && 
        leftSpeed < Config::MotorCorrection::CORRECTION_BAND_2_HIGH) {
      correctionLeft = Config::MotorCorrection::CORRECTION_BAND_2_FACTOR;
    }
    else if (leftSpeed > Config::MotorCorrection::CORRECTION_BAND_1_LOW && 
             leftSpeed < Config::MotorCorrection::CORRECTION_BAND_1_HIGH) {
      correctionLeft = Config::MotorCorrection::CORRECTION_BAND_1_FACTOR;
    }
    else if (leftSpeed < Config::MotorCorrection::CORRECTION_BAND_3_HIGH) {
      correctionLeft = Config::MotorCorrection::CORRECTION_BAND_3_FACTOR;
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
    leftSpeed = constrain(leftSpeed, 0, Config::Motor::MAX_SPEED);
    rightSpeed = constrain(rightSpeed, 0, Config::Motor::MAX_SPEED);
    
    // Apply minimum speed threshold
    if (leftSpeed > 0 && leftSpeed < Config::Motor::MIN_SPEED) leftSpeed = Config::Motor::MIN_SPEED;
    if (rightSpeed > 0 && rightSpeed < Config::Motor::MIN_SPEED) rightSpeed = Config::Motor::MIN_SPEED;
    
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
