#include "Robot.h"

Robot::Robot()
    : motors(Config::Motor::LEFT_IN1, Config::Motor::LEFT_IN2, Config::Motor::LEFT_ENA,
             Config::Motor::RIGHT_IN3, Config::Motor::RIGHT_IN4, Config::Motor::RIGHT_ENB),
      encoderLeft(Config::Encoder::LEFT_A, Config::Encoder::LEFT_B),
      encoderRight(Config::Encoder::RIGHT_A, Config::Encoder::RIGHT_B),
      yawPID(Config::YawPID::KP, Config::YawPID::KI, Config::YawPID::KD),
      speedScale(Config::Robot::SPEED_SCALE), turnCmd(Config::Robot::TURN_CMD_DEFAULT), kTurn(Config::Robot::K_TURN),
      lastControlUpdate(0), isRunning(false) {
}

bool Robot::initialize() {
    Serial.println("\n=== ROBOT INITIALIZATION (Caster Wheel - Yaw Only) ===\n");
    
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
    
    yawPID.initialize();
    delay(100);
    
    Serial.println("\n=== ROBOT READY ===\n");
    return true;
}

void Robot::update() {
    if (!isRunning) return;
    
    // Update IMU yaw only
    if (!imu.updateYaw()) {
        return;  // No new data
    }
    
    // Update encoder speeds
    encoderLeft.updateSpeed();
    encoderRight.updateSpeed();

    // If yaw control is enabled, compute PID output and apply to turnCmd
    if (yawControlEnabled) {
        double currentYaw = imu.getYawAngle();
        yawPID.update(currentYaw);
        turnCmd = yawPID.getOutput();
    }
    
    // ===== TASK EXECUTION STATE MACHINE =====
    switch (currentTaskState) {
        case TASK_MOVING_DISTANCE:
            updateDistanceTask();
            break;
        case TASK_ROTATING:
            updateRotationTask();
            break;
        case TASK_MOVING_TO_BALL:
            updateBallTrackingTask();
            break;
        case TASK_COMPLETE:
            // Task was complete, now idle
            currentTaskState = TASK_IDLE;
            motors.stop();
            break;
        case TASK_IDLE:
        default:
            break;
    }
    
    // ===== MOTOR CONTROL (simplified differential drive) =====
    updateMotorControl();
}

void Robot::updateDistanceTask() {
    int64_t currentLeftTicks = encoderLeft.getCount();
    int64_t currentRightTicks = encoderRight.getCount();
    
    int64_t leftDelta = currentLeftTicks - lastLeftTicks;
    int64_t rightDelta = currentRightTicks - lastRightTicks;
    int64_t avgDelta = (leftDelta + rightDelta) / 2;
    
    currentDistance += (double)avgDelta / Config::Robot::TICKS_PER_METER;
    lastLeftTicks = currentLeftTicks;
    lastRightTicks = currentRightTicks;
    
    // Check if target distance reached
    if (currentDistance >= targetDistance) {
        Serial.print(">>> DISTANCE TASK COMPLETE: ");
        Serial.print(currentDistance, 2);
        Serial.println(" meters");
        currentTaskState = TASK_COMPLETE;
        finish();
        return;
    }
}

void Robot::updateRotationTask() {
    if (!yawControlEnabled) {
        enableYawControl(true);
    }
    
    double currentYaw = imu.getYawAngle();
    double yawError = targetYaw - currentYaw;
    
    // Normalize error to shortest path
    while (yawError > 180.0) yawError -= 360.0;
    while (yawError < -180.0) yawError += 360.0;
    
    // Check if within tolerance
    if (fabs(yawError) <= yawTolerance) {
        Serial.print(">>> ROTATION TASK COMPLETE: Target ");
        Serial.print(targetYaw, 1);
        Serial.print("° | Current ");
        Serial.print(currentYaw, 1);
        Serial.println("°");
        enableYawControl(false);
        currentTaskState = TASK_COMPLETE;
        finish();
        return;
    }
}

void Robot::updateBallTrackingTask() {
    // Ball tracking is continuous - user calls moveToBall() with fresh commands
    // No automatic completion - call stopDistanceMovement() or another task to exit
}

void Robot::updateMotorControl() {
    // Simple differential drive control
    // Forward command controls base speed, turn command differentiates left/right
    
    unsigned long now = millis();
    if (now - lastControlUpdate >= Config::Robot::CONTROL_SAMPLE_TIME) {
        // Calculate motor speeds from forward and turn commands
        // speedScale controls overall speed, kTurn controls turn sensitivity
        double baseSpeed = speedScale * 100.0;  // Scale forward command to PWM
        double turnAmount = kTurn * turnCmd * 50.0;  // Scale turn command
        
        int leftSpeed = (int)(baseSpeed - turnAmount);
        int rightSpeed = (int)(baseSpeed + turnAmount);
        
        // Apply motor commands
        motors.move(leftSpeed, rightSpeed);
        
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

void Robot::setForwardCommand(double cmd) {
    // Direct forward speed control (0.0 to 1.0)
    speedScale = constrain(cmd, 0.0, 1.0);
}

void Robot::setTurnCommand(double cmd) {
    turnCmd = constrain(cmd, -1.0, 1.0);
}

void Robot::setYawSetpoint(double angle) {
    yawPID.setSetpoint(angle);
}

void Robot::enableYawControl(bool enable) {
    yawControlEnabled = enable;
    if (enable) {
        imu.resetYaw();  // Reset yaw tracking to 0
        yawPID.setSetpoint(0.0);
        Serial.println("Yaw control enabled!");
    } else {
        Serial.println("Yaw control disabled!");
    }
}

void Robot::setYawPIDGains(double Kp, double Ki, double Kd) {
    yawPID.setGains(Kp, Ki, Kd);
}

void Robot::setSpeedScale(double scale) {
    speedScale = scale;
}

void Robot::moveDistance(double meters) {
    if (!isRunning) {
        Serial.println("Robot not running! Call start() first.");
        return;
    }
    targetDistance = meters;
    currentDistance = 0.0;
    lastLeftTicks = encoderLeft.getCount();
    lastRightTicks = encoderRight.getCount();
    currentTaskState = TASK_MOVING_DISTANCE;
    Serial.print(">>> TASK: moveDistance(");
    Serial.print(meters, 2);
    Serial.println(" meters)");
}

void Robot::stopDistanceMovement() {
    if (currentTaskState == TASK_MOVING_DISTANCE) {
        currentTaskState = TASK_COMPLETE;
        finish();
    }
    motors.stop();
    Serial.println(">>> Distance movement stopped");
}

void Robot::finish() {
    Serial.println(" FIFO is done leave her alone");
}

void Robot::rotate(double degrees, bool relative) {
    if (!isRunning) {
        Serial.println("Robot not running! Call start() first.");
        return;
    }

    double currentYaw = imu.getYawAngle();
    double desiredYaw;

    if (relative) {
        desiredYaw = currentYaw + degrees;
    } else {
        desiredYaw = degrees;
    }

    // Normalize to -180..180
    while (desiredYaw > 180.0) desiredYaw -= 360.0;
    while (desiredYaw < -180.0) desiredYaw += 360.0;

    targetYaw = desiredYaw;
    // Apply setpoint and enable yaw control
    setYawSetpoint(targetYaw);
    yawControlEnabled = true; // enable control loop
    currentTaskState = TASK_ROTATING;

    Serial.print(">>> TASK: rotate(");
    Serial.print(degrees, 1);
    Serial.print(relative ? " deg (relative)" : " deg (absolute)");
    Serial.print(" | current=");
    Serial.print(currentYaw, 1);
    Serial.print("° -> target=");
    Serial.print(targetYaw, 1);
    Serial.println("°");
}

void Robot::moveToBall(double forwardCmd, double turnCmd) {
    if (!isRunning) {
        Serial.println("Robot not running! Call start() first.");
        return;
    }
    ballForwardCmd = forwardCmd;
    ballTurnCmd = turnCmd;
    currentTaskState = TASK_MOVING_TO_BALL;
    
    // Apply commands directly
    setForwardCommand(forwardCmd);
    setTurnCommand(turnCmd);
    
    Serial.print(">>> TASK: moveToBall(forward=");
    Serial.print(forwardCmd, 2);
    Serial.print(", turn=");
    Serial.print(turnCmd, 2);
    Serial.println(")");
}

void Robot::stopMoveToBall() {
    if (currentTaskState == TASK_MOVING_TO_BALL) {
        currentTaskState = TASK_COMPLETE;
        motors.stop();
        Serial.println(">>> MoveToBall task stopped — ball considered reached");
        finish();
    } else {
        Serial.println(">>> stopMoveToBall called but no move-to-ball task active");
    }
}

void Robot::printStatus() {
    Serial.println("\n=== ROBOT STATUS ===");
    Serial.print("Running: ");
    Serial.println(isRunning ? "YES" : "NO");
    Serial.print("Yaw: ");
    Serial.print(imu.getYawAngle(), 2);
    Serial.println("°");
    Serial.print("Left Speed: ");
    Serial.print(encoderLeft.getSpeedTicksPerSec(), 1);
    Serial.print(" ticks/s | Right Speed: ");
    Serial.print(encoderRight.getSpeedTicksPerSec(), 1);
    Serial.println(" ticks/s");
    if (currentTaskState == TASK_MOVING_DISTANCE) {
        Serial.print("Distance: ");
        Serial.print(currentDistance, 2);
        Serial.print(" / ");
        Serial.print(targetDistance, 2);
        Serial.println(" meters");
    }
}

void Robot::printDebug() {
    imu.printDiagnostics();
    if (yawControlEnabled) {
        yawPID.printDebug();
    }
}
