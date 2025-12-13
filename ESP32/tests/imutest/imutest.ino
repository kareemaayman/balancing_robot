#include <Wire.h>

#include "IMU.h"
IMU imu;

unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

   Wire.begin(21, 22, 400000);            // SDA, SCL default pins on ESP32
  imu.initialize();
}

void loop() {
imu.updateAngle();
imu.printDiagnostics();
}