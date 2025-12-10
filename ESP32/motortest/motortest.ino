#include "MotorDriver.h"

MotorDriver motors(26, 27, 25,   // left IN1, IN2, ENA
                   32, 23, 33);  // right IN3, IN4, ENB

void setup() {
  Serial.begin(115200);
  delay(1000);

  motors.initialize();

  Serial.println("Motor test starting");
}

void loop() {
  // 1) Both forward, ramp speed up
  Serial.println("Forward ramp");
  for (int s = 0; s <= 255; s += 20) {
    motors.move(s, s);    // both motors same speed
    delay(300);
  }

  // 2) Stop and pause
  motors.stop();
  delay(1000);

  // 3) Both backward, ramp speed up
  Serial.println("Backward ramp");
  for (int s = 0; s <= 255; s += 20) {
    motors.move(-s, -s);  // negative = reverse
    delay(300);
  }

  // 4) Stop and pause
  motors.stop();
  delay(2000);
  
  // 5) Both Right, ramp speed up
  Serial.println("Right ramp");
  for (int s = 0; s <= 255; s += 20) {
    motors.move(-s, s);    // both motors same speed
    delay(300);
  }
  
  // 6) Stop and pause
  motors.stop();
  delay(1000);
  
  // 7) Both Left, ramp speed up
  Serial.println("Left ramp");
  for (int s = 0; s <= 255; s += 20) {
    motors.move(s, -s);    // both motors same speed
    delay(300);
  }

  // 8) Stop and pause
  motors.stop();
  delay(1000);
}