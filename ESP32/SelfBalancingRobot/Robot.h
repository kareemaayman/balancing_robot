#ifndef ROBOT_H
#define ROBOT_H

#include "IMU.h"
#include "MotorDriver.h"
#include "EncoderReader.h"
#include "AnglePID.h"
#include "SpeedPID.h"

class Robot {
private:
    IMU imu;
    MotorDriver motors;
    EncoderReader encoderLeft;
    EncoderReader encoderRight;
    AnglePID anglePID;
    SpeedPID speedPIDLeft;
    SpeedPID speedPIDRight;
    
    // Cascaded control parameters
    double speedScale;
    double turnCmd;
    double kTurn;
    
    // Timing
    unsigned long lastControlUpdate;
    const unsigned long CONTROL_SAMPLE_TIME = 10;
    
    // State
    bool isRunning;
    
public:
    Robot();
    
    // Initialization
    bool initialize();
    
    // Main control loop
    void update();
    
    // Start/stop
    void start();
    void stop();
    bool running() const { return isRunning; }
    
    // Setpoints
    void setAngleSetpoint(double angle);
    void setForwardCommand(double cmd);
    void setTurnCommand(double cmd);
    
    // Tuning
    void setAnglePIDGains(double Kp, double Ki, double Kd);
    void setSpeedPIDGains(double Kp, double Ki, double Kd);
    void setSpeedScale(double scale);
    
    // Diagnostics
    void printStatus();
    void printDebug();
};

#endif
