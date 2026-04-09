#include "Robot.h"

Robot::Robot()
    : motors(Config::Motor::IN1, Config::Motor::IN2, Config::Motor::ENA,
             Config::Motor::IN3, Config::Motor::IN4, Config::Motor::ENB),
      encoderLeft(Config::Encoder::LEFT_A, Config::Encoder::LEFT_B),
      encoderRight(Config::Encoder::RIGHT_A, Config::Encoder::RIGHT_B),
      yawPID(Config::YawPID::KP, Config::YawPID::KI, Config::YawPID::KD),
      speedScale(Config::Robot::SPEED_SCALE), turnCmd(Config::Robot::TURN_CMD_DEFAULT), kTurn(Config::Robot::K_TURN),
      isRunning(false) {
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


void Robot::rotatePredefinedAngle(double degrees) {
    if (!isRunning) {
        Serial.println("Robot not running! Call start() first.");
        return;
    }

    // Get starting yaw
    // Wait for at least one valid DMP sample
    unsigned long waitStart = millis();
    while (!imu.updateYaw()) {
        if (millis() - waitStart > 500) {   // 0.5 s timeout
            Serial.println("IMU has no data yet (timeout)!");
            return;
        }
        delay(5);
    }

    imu.resetYawReference();
    double startYaw  = imu.getYawDegRelative(); // ~0
    double targetYaw = degrees;

    // Normalize target to -180..180
    //while (targetYaw > 180.0) targetYaw -= 360.0;
    //while (targetYaw < -180.0) targetYaw += 360.0;

    Serial.print(">>> ROTATE SIMPLE: ");
    Serial.print(degrees, 1);
    Serial.print(" deg | start=");
    Serial.print(startYaw, 1);
    Serial.print("° -> target=");
    Serial.print(targetYaw, 1);
    Serial.println("°");

    const double yawTolerance   = 10.0;      // stop when within ±5°
    const unsigned long TIMEOUT = 8000;     // 8 s safety timeout
    const unsigned long LOOP_DT = 20;       // 20 ms (same as your stable test)
    const int TURN_PWM          = 100;      // fixed spin speed

    unsigned long startTime    = millis();
    unsigned long lastLoopTime = millis();

    while (true) {
        unsigned long now = millis();

        // Run loop at ~50 Hz (like the stable test)
        if (now - lastLoopTime < LOOP_DT) {
            continue;
        }
        lastLoopTime = now;

        // Safety timeout
        if (now - startTime > TIMEOUT) {
            Serial.println(">>> ROTATE TIMEOUT");
            break;
        }

        // Get new yaw (same pattern as stable test)
        if (!imu.updateYaw()) {
            continue;
        }
        double yaw = imu.getYawDegRelative();  // starts near 0.0

        double err = targetYaw - yaw;
        //while (err > 180.0)  err -= 360.0;
        //while (err < -180.0) err += 360.0;

        Serial.print("yaw=");
        Serial.print(yaw, 1);
        Serial.print(" target=");
        Serial.print(targetYaw, 1);
        Serial.print(" err=");
        Serial.println(err, 1);

        // Stop if within tolerance
        if (fabs(err) <= yawTolerance) {
            Serial.println(">>> ROTATE DONE");
            break;
        }

        // Spin direction based on requested sign
        if (degrees > 0) {
            // spin left in place; flip signs if direction is wrong
            motors.move( TURN_PWM, -TURN_PWM);
        } else if (degrees < 0) {
            // spin right in place
            motors.move(-TURN_PWM,  TURN_PWM);
        } else {
            break;
        }
    }

    motors.stop();
    finish();
}

void Robot::moveDistance(double meters, double speed) {
    if (!isRunning) {
        Serial.println("Robot not running! Call start() first.");
        return;
    }

    Serial.print(">>> MOVE DISTANCE: ");
    Serial.print(meters, 2);
    Serial.println(" m");
    // Reset encoders FIRST
    encoderLeft.reset();
    encoderRight.reset();
    // Initialize local variables
    double distanceTraveled = 0.0;
    //int64_t lastLeftTicks  = encoderLeft.getCount();
    //int64_t lastRightTicks = encoderRight.getCount();

    // Forward command (-1..1)
    setForwardCommand(speed);
    setTurnCommand(0.0);

    unsigned long lastControlUpdate = 0;
    unsigned long lastDistanceUpdate = millis();

    // Blocking loop
    while (distanceTraveled < meters) {
        unsigned long now = millis();
        // 1) Update distance at fixed rate (e.g. 50Hz)
        if (now - lastDistanceUpdate >= 20) {  // 50Hz
            int64_t leftTicks  = encoderLeft.getCount();   // Read ONCE
            int64_t rightTicks = encoderRight.getCount();
            
            // Use total ticks (no delta, since we reset)
            distanceTraveled = (double)(leftTicks + rightTicks) / 2.0 / Config::Robot::TICKS_PER_METER;
            
            lastDistanceUpdate = now;
            Serial.print("Distance: "); Serial.print(distanceTraveled, 3); Serial.println(" m");
        }
        
        // 2) Motor control at fixed rate
        if (now - lastControlUpdate >= Config::Robot::CONTROL_SAMPLE_TIME) {
            double baseSpeed = speed * 255.0;
            int leftSpeed  = constrain((int)baseSpeed, -255, 255);
            int rightSpeed = constrain((int)baseSpeed, -255, 255);
            motors.move(leftSpeed, rightSpeed);
            lastControlUpdate = now;
        }
        
        // Safety timeout
        if (now > 30000) {  // 30 sec max
            Serial.println("TIMEOUT!");
            break;
        }
    }

    motors.stop();
    Serial.print(">>> DISTANCE DONE: ");
    Serial.print(distanceTraveled, 2);
    Serial.println(" m");
    finish();
}

void Robot::driveCommand(double forwardCmd, double turnCmd) {
    if (!isRunning) {
        Serial.println("Robot not running! Call start() first.");
        return;
    }

    // Save commands
    setForwardCommand(forwardCmd);  // -1..1
    setTurnCommand(turnCmd);        // -1..1

    // Simple differential control (no state machine)
    double baseSpeed = speedScale * 255.0;
    double turnAmount = kTurn * this->turnCmd * 255.0;

    int leftSpeed  = (int)(baseSpeed + turnAmount);
    int rightSpeed = (int)(baseSpeed - turnAmount);

    motors.move(leftSpeed, rightSpeed);

    Serial.print(">>> DRIVE CMD: fwd=");
    Serial.print(forwardCmd, 2);
    Serial.print(" turn=");
    Serial.print(turnCmd, 2);
    Serial.print(" | L=");
    Serial.print(leftSpeed);
    Serial.print(" R=");
    Serial.println(rightSpeed);
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
    speedScale = constrain(cmd, -1.0, 1.0);
}

void Robot::setTurnCommand(double cmd) {
    turnCmd = constrain(cmd, -1.0, 1.0);
}

void Robot::setYawPIDGains(double Kp, double Ki, double Kd) {
    yawPID.setGains(Kp, Ki, Kd);
}

void Robot::finish() {
    Serial.println(" FIFO is done, leave her alone ");
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
}

void Robot::printDebug() {
    imu.printDiagnostics();
    if (yawControlEnabled) {
        yawPID.printDebug();
    }
}
