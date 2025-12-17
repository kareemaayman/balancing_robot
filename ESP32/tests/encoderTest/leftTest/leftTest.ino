#include "EncoderReader.h"
#include "MotorDriver.h"

MotorDriver motors(26, 27, 25,   // left IN1, IN2, ENA
                   32, 23, 33);  // right IN3, IN4, ENB
EncoderReader left(18, 19);
unsigned long lastPrint = 0;
void setup() {
    Serial.begin(115200);
    motors.initialize();
    left.initialize();
    left.reset();
  
}

void loop() {
  motors.move(200, 200);
  // Update speed estimation
  left.updateSpeed();
  unsigned long now = millis();
  if (now - lastPrint >= 50) {
    lastPrint = now;
    // Print raw data
    left.printDebug();   // prints Count, Delta, Speed ticks/s
  }
}