// Self-Balancing Robot Main Sketch
// Using modular library architecture

#include "Robot.h"

Robot robot;
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 200;  // Print every 200ms

void setup() {
    Serial.begin(115200);
    delay(2000);  // Give serial time to initialize
    
    Serial.println("\n\n=== SELF-BALANCING ROBOT ===\n");
    
    // Initialize robot and all components
    if (!robot.initialize()) {
        Serial.println("FATAL: Robot initialization FAILED!");
        while(1) {
            delay(1000);
        }
    }
    
    // Configure PID gains (conservative starting values)
    robot.setAnglePIDGains(20.0, 0.0, 0.5);
    robot.setSpeedPIDGains(0.5, 0.2, 0.1);
    
    // Start robot
    robot.start();
    lastPrintTime = millis();
}

void loop() {
    // Main control update - happens every ~10ms
    robot.update();
    
    // Print diagnostics every 200ms for monitoring
    if (millis() - lastPrintTime > PRINT_INTERVAL) {
        robot.printDebug();
        Serial.println("---");
        lastPrintTime = millis();
    }
    
    // Optional: Handle serial commands for tuning (implement as needed)
    // handleSerialCommands();
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
