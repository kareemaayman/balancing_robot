#include <Arduino.h>
#include "MotorDriver.h"
#include "EncoderReader.h"

MotorDriver motors(26, 27, 25, 32, 23, 33);
EncoderReader encL(34, 35);
EncoderReader encR(39, 36);

const int TEST_PWM = 150;
const int TEST_TIME = 5000;  // 5 sec

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
  
  motors.move(TEST_PWM, TEST_PWM);
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
    sumL += encL.getSpeedTicksPerSec();
    sumR += encR.getSpeedTicksPerSec();
    count++;
        // Print every 500ms
    if (now - lastPrint >= 500) {
      lastPrint = now;
      Serial.print("L: ");
      Serial.print(speedL, 1);
      Serial.print(" ticks/s | R: ");
      Serial.print(speedR, 1);
      Serial.println(" ticks/s");
    }
    delay(100);
  } else if (count > 0 && elapsed > TEST_TIME) {
    motors.move(0, 0);
    
    double avgL = sumL / count;
    double avgR = sumR / count;
    Serial.println("\n=== AVERAGE ===");
    Serial.print("L: ");
    Serial.print(avgL, 2);
    Serial.print(" R: ");
    Serial.println(avgR, 2);
    
    if (avgL > avgR) {
      Serial.print("corrLeft = ");
      Serial.println(avgR / avgL, 4);
    } else if (avgL < avgR){
      Serial.print("corrRight = ");
      Serial.println(avgL / avgR, 4);
    } else {
      Serial.print(" motors synchronized ");

    }
    
    while(1);  // Stop
  }
}
