#include "Robot.h"

Robot::Robot()
    : motors(26, 27, 25, 32, 23, 33),  // IN1, IN2, ENA, IN3, IN4, ENB
      encoderLeft(34, 35),      // LA, LB
      encoderRight(39, 36),     // RA, RB
      anglePID(20.0, 0.0, 0.5),      // Kp,Ki,Kd
      speedPIDLeft("Left", 0.5, 0.2, 0.1), //direction,Kp,Ki,Kd
      speedPIDRight("Right", 0.5, 0.2, 0.1),
      speedScale(5.0), turnCmd(0), kTurn(20.0),
      lastControlUpdate(0), isRunning(false) {
    speedPIDLeft.initialize();
    speedPIDRight.initialize();
}

bool Robot::initialize() {
    Serial.println("\n=== ROBOT INITIALIZATION ===\n");
    
    if (!imu.initialize()) {
        Serial.println("IMU initialization failed!");
        return false;
    }
    delay(100);
    
    motors.initialize();
    delay(100);
    
    encoderLeft.initialize();
    encoderRight.initialize();
    delay(100);
    
    anglePID.initialize();
    delay(100);
    
    Serial.println("\n=== ROBOT READY ===\n");
    return true;
}

void Robot::update() {
    if (!isRunning) return;
    
    // Update IMU angle
    if (!imu.updateAngle()) {
        return;  // No new data
    }
    
    // Update encoder speeds
    encoderLeft.updateSpeed();
    encoderRight.updateSpeed();
    
    // Update angle PID (outer loop)
    anglePID.update(imu.getPitchAngle());
    double angleOutput = anglePID.getOutput();
    
    unsigned long now = millis();
    if (now - lastControlUpdate >= CONTROL_SAMPLE_TIME) {
        // Convert angle error to speed setpoints
        double speedSetpointLeft = (angleOutput * speedScale) - (kTurn * turnCmd * 10);
        double speedSetpointRight = (angleOutput * speedScale) + (kTurn * turnCmd * 10);
        
        // Update speed PID (inner loop)
        speedPIDLeft.setSetpoint(speedSetpointLeft);
        speedPIDLeft.setInput(encoderLeft.getSpeedTicksPerSec());
        speedPIDLeft.update();
        
        speedPIDRight.setSetpoint(speedSetpointRight);
        speedPIDRight.setInput(encoderRight.getSpeedTicksPerSec());
        speedPIDRight.update();
        
        // Send to motors
        int leftMotorPWM = (int)speedPIDLeft.getOutput();
        int rightMotorPWM = (int)speedPIDRight.getOutput();
        motors.move(leftMotorPWM, rightMotorPWM);
        
        lastControlUpdate = now;
    }
}

void Robot::start() {
    isRunning = true;
    Serial.println("Robot started!");
}

void Robot::stop() {
    isRunning = false;
    motors.stop();
    Serial.println("Robot stopped!");
}

void Robot::setAngleSetpoint(double angle) {
    anglePID.setSetpoint(angle);
}

void Robot::setForwardCommand(double cmd) {
    anglePID.setForwardCommand(cmd);
}

void Robot::setTurnCommand(double cmd) {
    turnCmd = constrain(cmd, -1.0, 1.0);
}

void Robot::setAnglePIDGains(double Kp, double Ki, double Kd) {
    anglePID.setGains(Kp, Ki, Kd);
}

void Robot::setSpeedPIDGains(double Kp, double Ki, double Kd) {
    speedPIDLeft.setGains(Kp, Ki, Kd);
    speedPIDRight.setGains(Kp, Ki, Kd);
}

void Robot::setSpeedScale(double scale) {
    speedScale = scale;
}

void Robot::printStatus() {
    Serial.println("\n=== ROBOT STATUS ===");
    Serial.print("Running: ");
    Serial.println(isRunning ? "YES" : "NO");
    Serial.print("Angle: ");
    Serial.print(imu.getPitchAngle(), 2);
    Serial.println("°");
    Serial.print("Left Speed: ");
    Serial.print(encoderLeft.getSpeedTicksPerSec(), 1);
    Serial.print(" ticks/s (");
    Serial.print("Right Speed: ");
    Serial.print(encoderRight.getSpeedTicksPerSec(), 1);
    Serial.print(" ticks/s (");
}

void Robot::printDebug() {
    imu.printDiagnostics();
    anglePID.printDebug();
    speedPIDLeft.printDebug();
    speedPIDRight.printDebug();
}
