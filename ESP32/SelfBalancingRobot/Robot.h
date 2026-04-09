#ifndef ROBOT_H
#define ROBOT_H

#include "Config.h"
#include "IMU.h"
#include "MotorDriver.h"
#include "EncoderReader.h"
#include "YawPID.h"

class Robot {
private:
    IMU imu;
    MotorDriver motors;
    EncoderReader encoderLeft;
    EncoderReader encoderRight;
    YawPID yawPID;

    // Control parameters
    double speedScale;
    double turnCmd;
    double kTurn;

    // Yaw control
    bool yawControlEnabled = false;
    double targetYaw = 0.0;
    double yawTolerance = 2.0;

    // State
    bool isRunning;

public:
    Robot();

    // Initialization
    bool initialize();

    // Start/stop
    void start();
    void stop();
    bool running() const { return isRunning; }

    // Direct command setters
    void setForwardCommand(double cmd);
    void setTurnCommand(double cmd);

    // High-level blocking actions
    void moveDistance(double meters, double speed);
    void rotatePredefinedAngle(double degrees);

    // Direct differential drive command (for ROS etc.)
    void driveCommand(double forwardCmd, double turnCmd);

    // Tuning
    void setYawPIDGains(double Kp, double Ki, double Kd);

    // Diagnostics
    void printStatus();
    void printDebug();

    // Called when any blocking action finishes
    void finish();
};

#endif
