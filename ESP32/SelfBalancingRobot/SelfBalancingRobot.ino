// Self-Balancing Robot Main Sketch
// Using modular library architecture

#include "Config.h"
#include "Robot.h"

Robot robot;
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = Config::Serial_Config::PRINT_INTERVAL;

// Sequence of tasks to execute
int taskIndex = 0;
bool tasksInitialized = false;

void setup() {
    Serial.begin(Config::Serial_Config::BAUD_RATE);
    delay(Config::Serial_Config::STARTUP_DELAY);
    
    Serial.println("\n\n=== SELF-BALANCING ROBOT ===\n");
    
    // Initialize robot and all components
    if (!robot.initialize()) {
        Serial.println("FATAL: Robot initialization FAILED!");
        while(1) {
            delay(1000);
        }
    }
    
    // Start robot
    robot.start();
    lastPrintTime = millis();
    
    // === TASK SEQUENCE DEFINITION ===
    // Uncomment ONE sequence to test, or create your own
    
    // Sequence 1: Simple movement test
    // robot.moveDistance(1.0);
    
    // Sequence 2: Rotation test  
    // robot.rotatePredefinedAngle(90.0);
    
    // Sequence 3: Ball tracking test
    // robot.moveToBall(0.3, 0.1);
}

void loop() {
    // Main control update - happens every ~10ms
    robot.update();
    
    // Print diagnostics every 200ms for monitoring
    if (millis() - lastPrintTime > PRINT_INTERVAL) {
        printTaskStatus();
        robot.printDebug();
        Serial.println("---");
        lastPrintTime = millis();
    }
}

void printTaskStatus() {
    TaskState state = robot.getTaskState();
    Serial.print("[TASK STATE] ");
    
    switch(state) {
        case TASK_IDLE:
            Serial.println("IDLE");
            break;
        case TASK_MOVING_DISTANCE:
            Serial.print("MOVING_DISTANCE: ");
            Serial.print(robot.getDistanceTraveled(), 2);
            Serial.print(" / ");
            Serial.print(robot.getTargetDistance(), 2);
            Serial.println(" meters");
            break;
        case TASK_ROTATING:
            Serial.print("ROTATING: Current=");
            Serial.print(robot.getCurrentYaw(), 1);
            Serial.print("°, Target=");
            Serial.print(robot.getTargetYaw(), 1);
            Serial.println("°");
            break;
        case TASK_MOVING_TO_BALL:
            Serial.println("MOVING_TO_BALL");
            break;
        case TASK_COMPLETE:
            Serial.println("COMPLETE");
            break;
        default:
            Serial.println("UNKNOWN");
    }
}

// Future: Implement serial command handler for real-time PID tuning
/*
void handleSerialCommands() {
    if (Serial.available()) {
        char cmd = Serial.read();
        
        if (cmd == 'S') {
            robot.start();
        } else if (cmd == 's') {
            robot.stop();
        } else if (cmd == 'T') {
            robot.printStatus();
        }
        // Add more commands as needed
    }
}
*/
