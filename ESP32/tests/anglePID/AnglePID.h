#ifndef ANGLE_PID_H
#define ANGLE_PID_H

#include <Arduino.h>
#include <PID_v1.h>

class AnglePID {
private:
    double upright_angle = 90.0;
    double setpoint;
    double input;
    double output;
    double Kp, Ki, Kd;
    
    PID pid;
    
    unsigned long lastComputeTime;
    const unsigned long SAMPLE_TIME_MS = 10;
    
    // Command inputs
    double forwardCmd=0;
    double kMove;
    
public:
    AnglePID(double Kp = 20.0, double Ki = 0.0, double Kd = 0.5);
    
    void initialize();
    
    // Set target angle
    void setSetpoint(double angle);
    double getSetpoint() const { return setpoint; }
    
    // Update with current angle
    void update(double currentAngle);
    double getOutput() const { return output; }
    
    // Motion commands
    void setForwardCommand(double cmd);
    void setMoveGain(double k);
    
    // PID tuning
    void setGains(double Kp, double Ki, double Kd);
    void printDebug();
};

#endif
