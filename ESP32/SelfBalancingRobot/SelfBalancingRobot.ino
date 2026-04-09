#include "Config.h"
#include "Robot.h"

Robot robot;
//MotorDriver motors(Config::Motor::IN1, Config::Motor::IN2, Config::Motor::ENA,
  //  Config::Motor::IN3, Config::Motor::IN4, Config::Motor::ENB);

// ====== TEST CONFIG ======
// Uncomment exactly ONE of these at a time to test

//#define TEST_MOVE_DISTANCE
//#define TEST_ROTATE_ANGLE
#define TEST_MANUAL_DRIVE   // simple forward/backward/turn demo

void setup() {
    Serial.begin(Config::Serial_Config::BAUD_RATE);
    delay(Config::Serial_Config::STARTUP_DELAY);

    Serial.println("\n=== ROBOT TEST SKETCH ===");
    
    if (!robot.initialize()) {
        Serial.println("Robot init FAILED!");
        while (true) { delay(1000); }
    }

    robot.start();
    Serial.println("Robot init DONE, starting test...");
    
#ifdef TEST_MOVE_DISTANCE
    Serial.println(">> TEST: MOVE DISTANCE");
    // Example: move 1.0 m at 50% speed
    robot.moveDistance(1.0, 0.5);
    Serial.println(">> MOVE DISTANCE TEST DONE");
#endif

#ifdef TEST_ROTATE_ANGLE
    Serial.println(">> TEST: ROTATE ANGLE");
    // Example: rotate +90 degrees (clockwise)
    
    robot.rotatePredefinedAngle(-45.0);
    //delay(1000);
    // Example: rotate -90 degrees (back)
    //robot.rotatePredefinedAngle(-90.0);
    Serial.println(">> ROTATE ANGLE TEST DONE");
#endif

#ifdef TEST_MANUAL_DRIVE
    Serial.println(">> TEST: MANUAL DRIVE");
    // Simple sequence: forward, stop, turn in place, stop, backward
    // Forward 0.4 for 2 seconds
    robot.driveCommand(0.4, 0.0);
    delay(2000);
    robot.driveCommand(0.0, 0.0);
    delay(1000);

    // Turn in place to the left for 2 seconds
    robot.driveCommand(0.0, 0.5);
    delay(2000);
    robot.driveCommand(0.0, 0.0);
    delay(1000);

    // Backwards (negative forward command not supported by your Robot,
    // so we just use small forward + negative turn to see different motion)
    robot.driveCommand(-0.3, -0.5);
    delay(2000);
    robot.driveCommand(0.0, 0.0);
    Serial.println(">> MANUAL DRIVE TEST DONE");
#endif

    robot.stop();
}

void loop() {
    //motors.move(120,120);
   // delay(1000);
   // motors.move(0,-200);
    //motors.move(200,0);
    //delay(1000);
   // motors.move(-200,0);
    // For now, all tests are blocking in setup(), so loop is empty.
    // Later you can move tests here for non-blocking / ROS-driven behavior.
}