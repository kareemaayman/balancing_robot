#include <Wire.h>
#include "IMU.h"

IMU imu;
double angleSum = 0;
int sampleCount = 0;
unsigned long startTime = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22, 400000);  // SDA, SCL, 400kHz
  
  if (!imu.initialize()) {
    Serial.println("ERROR: IMU init failed!");
    while(1);
  }

  Serial.println("\n=== IMU UPRIGHT ANGLE CALIBRATION ===");
  Serial.println("Hold robot perfectly upright (vertical).");
  Serial.println("Collecting samples for 5 seconds...\n");
  
  startTime = millis();
}

void loop() {
  imu.updateAngle();
  double angle = imu.getPitchAngle();

  angleSum += angle;
  sampleCount++;

  // Collect for 5 seconds
  unsigned long elapsed = millis() - startTime;
  if (elapsed >= 5000) {
    // Calculate average
    double avgAngle = angleSum / sampleCount;

    Serial.println("\n=== CALIBRATION COMPLETE ===");
    Serial.print("Samples collected: ");
    Serial.println(sampleCount);
    Serial.print("Average angle: ");
    Serial.println(avgAngle, 2);
    Serial.println("\nUpdate BalanceConfig.h:");
    Serial.print("  upright_angle = ");
    Serial.print(avgAngle, 1);
    Serial.println(";");
    Serial.println("\nThen recompile and upload.");

    while(1);  // Stop here
  }

  // Print live feedback
  Serial.print("Angle: ");
  Serial.print(angle, 2);
  Serial.print("° | Avg so far: ");
  Serial.println(angleSum / sampleCount, 2);

  delay(100);
}
