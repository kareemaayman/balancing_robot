#include <Arduino.h>
#include "MotorDriver.h"
#include "EncoderReader.h"

MotorDriver motors(26, 27, 25, 32, 23, 33);
EncoderReader encL(5,15);
EncoderReader encR(18,19);  // What are your right encoder pins?

int testPwm = 0;
const int TEST_TIME = 5000;

double sumL = 0, sumR = 0;
int count = 0;
unsigned long testStartTime = 0;
unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  motors.initialize();
  encL.initialize();
  encR.initialize();

  Serial.println("\nEnter TEST PWM (0–255) and press Enter:");

  // Wait for valid PWM
  while (testPwm == 0) {
    if (Serial.available()) {
      testPwm = Serial.parseInt();
      if (testPwm <= 0 || testPwm > 255) {
        Serial.println("Invalid PWM. Enter 1–255:");
        testPwm = 0;
      }
    }
  }

  Serial.print("Starting test with PWM = ");
  Serial.println(testPwm);

  motors.move(testPwm, testPwm);
  Serial.println("Testing...");
  
  testStartTime = millis();
  lastPrint = testStartTime;
}

void loop() {
  encL.updateSpeed();
  encR.updateSpeed();
  
  unsigned long now = millis();
  unsigned long elapsed = now - testStartTime;
  
  if (elapsed < TEST_TIME) {
    double speedL = encL.getSpeedTicksPerSec();
    double speedR = encR.getSpeedTicksPerSec();
    
    sumL += speedL;
    sumR += speedR;
    count++;
    
    if (now - lastPrint >= 500) {
      lastPrint = now;
      Serial.print("L: ");
      encL.printDebug(); 
      Serial.print(" | R: ");
      encR.printDebug(); 
      Serial.println(" ticks/s");
    }
    
    delay(100);
  } else if (count > 0 && elapsed >= TEST_TIME) {
    motors.move(0, 0);
    
    double avgL = abs(sumL / count);
    double avgR = abs(sumR / count);
    
    Serial.println("\n=== AVERAGE ===");
    Serial.print("L: ");
    Serial.print(avgL, 2);
    Serial.print(" R: ");
    Serial.println(avgR, 2);
    
    if (avgL > avgR) {
      Serial.print("motors.setCorrection(");
      Serial.print(avgR / avgL, 4);
      Serial.println(", 1.0);");
    } else {
      Serial.print("motors.setCorrection(1.0, ");
      Serial.print(avgL / avgR, 4);
      Serial.println(");");
    }
    while (1);
  }

}
