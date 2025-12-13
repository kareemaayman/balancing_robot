#include <Wire.h>
#include "MPU6050.h"
#include "Kalman.h"

MPU6050 mpu;

Kalman kalmanPitch;  // For pitch

unsigned long timer;
float pitch;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  delay(1000);

  Serial.println("Initializing MPU6050...");
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  Serial.println("MPU6050 ready.");
  timer = micros();
}

void loop() {
  // Read raw MPU6050 data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert gyro to deg/s
  float gyroXrate = gx / 131.0; // FS ±250°/s
  float gyroYrate = gy / 131.0;
  float gyroZrate = gz / 131.0;

  // Calculate pitch and roll from accelerometer
  float pitchAcc = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;  // Forward/back tilt

  // Calculate delta time
  unsigned long now = micros();
  float dt = (now - timer) / 1000000.0;
  timer = now;

  // Apply Kalman filter
  pitch = kalmanPitch.getAngle(pitchAcc, gyroYrate, dt);

  // Print every 100 ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    Serial.print("Pitch: "); Serial.println(pitch);

    lastPrint = millis();
  }

  delay(2); // Small delay to reduce I2C stress
}