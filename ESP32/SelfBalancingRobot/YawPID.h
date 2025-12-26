#ifndef YAW_PID_H
#define YAW_PID_H

#include <Arduino.h>
#include <PID_v1.h>
#include "Config.h"

class YawPID {
private:
    double setpoint;      // Target yaw angle in degrees
    double input;         // Current yaw angle in degrees
    double output;        // Turn command output (-1 to 1)
    double Kp, Ki, Kd;
    
    PID pid;
    
    unsigned long lastComputeTime;
    const unsigned long SAMPLE_TIME_MS = Config::YawPID::SAMPLE_TIME_MS;

public:
    YawPID(double Kp = Config::YawPID::KP, 
           double Ki = Config::YawPID::KI, 
           double Kd = Config::YawPID::KD);
    
    void initialize();
    
    // Set target yaw angle
    void setSetpoint(double angle);
    double getSetpoint() const { return setpoint; }
    
    // Update with current yaw angle
    void update(double currentYaw);
    double getOutput() const { return output; }
    
    // PID tuning
    void setGains(double Kp, double Ki, double Kd);
    void printDebug();
};

#endif
