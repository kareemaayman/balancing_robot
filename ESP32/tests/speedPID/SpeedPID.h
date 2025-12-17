#ifndef SPEED_PID_H
#define SPEED_PID_H

#include <Arduino.h>
#include <PID_v1.h>

class SpeedPID {
private:
    double setpoint;
    double input;
    double output;
    
    PID pid;
    
    unsigned long lastComputeTime;
    const unsigned long SAMPLE_TIME_MS = 10;
    
    const char* name;
    
public:
    SpeedPID(const char* name = "SpeedPID",
             double Kp = 0.5, double Ki = 0.2, double Kd = 0.1);
    
    void initialize();
    
    void setSetpoint(double speed);
    void setInput(double speed);
    double getOutput() const { return output; }
    
    void update();
    void setGains(double Kp, double Ki, double Kd);
    
    void printDebug();
};

#endif
