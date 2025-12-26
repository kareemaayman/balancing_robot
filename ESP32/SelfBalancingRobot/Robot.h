#ifndef ROBOT_H
#define ROBOT_H

#include "Config.h"
#include "IMU.h"
#include "MotorDriver.h"
#include "EncoderReader.h"
#include "YawPID.h"

// Task state enum for explicit state tracking
enum TaskState {
    TASK_IDLE,
    TASK_MOVING_DISTANCE,
    TASK_ROTATING,
    TASK_MOVING_TO_BALL,
    TASK_COMPLETE
};

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
    
    // Task state machine
    TaskState currentTaskState = TASK_IDLE;
    TaskState lastTaskState = TASK_IDLE;
    
    // Distance-based movement tracking
    double targetDistance = 0.0;
    double currentDistance = 0.0;
    int64_t lastLeftTicks = 0;
    int64_t lastRightTicks = 0;
    
    // Rotation tracking
    double targetYaw = 0.0;
    double yawTolerance = 2.0;
    
    // Ball tracking
    double ballForwardCmd = 0.0;
    double ballTurnCmd = 0.0;
    
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
    
    // Direct control
    void setForwardCommand(double cmd);
    void setTurnCommand(double cmd);
    
    // Yaw control
    void setYawSetpoint(double angle);
    void enableYawControl(bool enable);
    
    // Distance-based movement
    void moveDistance(double meters);
    void stopDistanceMovement();
    double getDistanceTraveled() const { return currentDistance; }
    
    // High-level autonomous functions
    void rotatePredefinedAngle(double degrees);
    void rotateBy(double degrees); // relative rotation: rotate current yaw + degrees
    void moveToBall(double forwardCmd, double turnCmd);
    void stopMoveToBall();
    
    // Task status queries
    TaskState getTaskState() const { return currentTaskState; }
    bool isTaskActive() const { return currentTaskState != TASK_IDLE && currentTaskState != TASK_COMPLETE; }
    bool isTaskComplete() const { return currentTaskState == TASK_COMPLETE; }
    double getTargetDistance() const { return targetDistance; }
    double getTargetYaw() const { return targetYaw; }
    double getCurrentYaw() const { return imu.getYawAngle(); }
    
    // Tuning
    void setYawPIDGains(double Kp, double Ki, double Kd);
    void setSpeedScale(double scale);
    
    // Diagnostics
    void printStatus();
    void printDebug();
    // Called when any task finishes
    void finish();
    
    // Internal task functions
    void updateDistanceTask();
    void updateRotationTask();
    void updateBallTrackingTask();
    void updateMotorControl();
};

#endif
