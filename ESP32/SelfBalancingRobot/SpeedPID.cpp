#include "SpeedPID.h"

SpeedPID::SpeedPID(const char* name, double Kp, double Ki, double Kd)
    : name(name), setpoint(0), input(0), output(0),
      pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT),
      lastComputeTime(0) {}

void SpeedPID::initialize() {
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(SAMPLE_TIME_MS);
    pid.SetOutputLimits(-255, 255);
}

void SpeedPID::setSetpoint(double speed) {
    setpoint = speed;
}

void SpeedPID::setInput(double speed) {
    input = speed;
}

void SpeedPID::update() {
    unsigned long now = millis();
    if (now - lastComputeTime >= SAMPLE_TIME_MS) {
        pid.Compute();
        lastComputeTime = now;
    }
}

void SpeedPID::setGains(double Kp, double Ki, double Kd) {
    pid.SetTunings(Kp, Ki, Kd);
}

void SpeedPID::printDebug() {
    Serial.print(name);
    Serial.print(" Setpoint: ");
    Serial.print(setpoint, 0);
    Serial.print(" | Input: ");
    Serial.print(input, 0);
    Serial.print(" | Output: ");
    Serial.println(output, 1);
}
