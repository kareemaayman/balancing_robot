#include "YawPID.h"

YawPID::YawPID(double Kp, double Ki, double Kd)
    : setpoint(Config::YawPID::SETPOINT_DEFAULT), 
      input(0), 
      output(0),
      Kp(Kp), 
      Ki(Ki), 
      Kd(Kd),
      pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT),
      lastComputeTime(0) {}

void YawPID::initialize() {
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(SAMPLE_TIME_MS);
    pid.SetOutputLimits(-1.0, 1.0);  // Output as turn command (-1 to 1)
    Serial.println("Yaw PID initialized!");
}

void YawPID::setSetpoint(double angle) {
    setpoint = angle;
}

void YawPID::update(double currentYaw) {
    input = currentYaw;
    
    // Normalize angle difference to shortest path
    double angleDiff = setpoint - input;
    while (angleDiff > 180.0) angleDiff -= 360.0;
    while (angleDiff < -180.0) angleDiff += 360.0;
    input = setpoint - angleDiff;
    
    // Only compute at sample rate
    unsigned long now = millis();
    if (now - lastComputeTime >= SAMPLE_TIME_MS) {
        pid.Compute();
        lastComputeTime = now;
    }
}

void YawPID::setGains(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    pid.SetTunings(Kp, Ki, Kd);
}

void YawPID::printDebug() {
    Serial.print("Yaw: ");
    Serial.print(input, 2);
    Serial.print("° | Setpoint: ");
    Serial.print(setpoint, 2);
    Serial.print("° | Turn Cmd: ");
    Serial.println(output, 3);
}
