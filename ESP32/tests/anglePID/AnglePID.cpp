#include "AnglePID.h"

AnglePID::AnglePID(double Kp, double Ki, double Kd)
    : setpoint(upright_angle), input(upright_angle), output(0),
      Kp(Kp), Ki(Ki), Kd(Kd),
      pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT),
      lastComputeTime(0), forwardCmd(0), kMove(2.0) {}

void AnglePID::initialize() {
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(SAMPLE_TIME_MS);
    pid.SetOutputLimits(-255, 255);
    Serial.println("Angle PID initialized!");
}

void AnglePID::setSetpoint(double angle) {
    setpoint = angle;
}

void AnglePID::update(double currentAngle) {
    input = currentAngle;
    
    // Apply forward command to setpoint
    setpoint = upright_angle + kMove * forwardCmd;
    
    // Only compute at sample rate
    unsigned long now = millis();
    if (now - lastComputeTime >= SAMPLE_TIME_MS) {
        pid.Compute();
        lastComputeTime = now;
    }
}

void AnglePID::setForwardCommand(double cmd) {
    forwardCmd = constrain(cmd, -1.0, 1.0);
}

void AnglePID::setMoveGain(double k) {
    kMove = k;
}

void AnglePID::setGains(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    pid.SetTunings(Kp, Ki, Kd);
}

void AnglePID::printDebug() {
    Serial.print("Angle: ");
    Serial.print(input, 2);
    Serial.print("° | Setpoint: ");
    Serial.print(setpoint, 2);
    Serial.print("° | Output: ");
    Serial.println(output, 1);
}
