#include "EncoderReader.h"
#include "MotorDriver.h"
#include "Config.h"
EncoderReader leftEncoder(Config::Encoder::LEFT_A, Config::Encoder::LEFT_B);
EncoderReader rightEncoder(Config::Encoder::RIGHT_A, Config::Encoder::RIGHT_B);
MotorDriver motors(Config::Motor::IN1, Config::Motor::IN2, Config::Motor::ENA,
                  Config::Motor::IN3, Config::Motor::IN4, Config::Motor::ENB);
const int TEST_PWM = 120;            // test speed, both wheels same
const unsigned long RUN_MS = 3000;   // drive duration in ms

void setup() {
  Serial.begin(115200);
  delay(1000);

  motors.initialize();
  leftEncoder.initialize();
  rightEncoder.initialize();

  Serial.println("Ticks-per-meter test: robot will drive straight.");

  // Reset encoders
  leftEncoder.reset();
  rightEncoder.reset();
  delay(200);

  // Drive forward
  motors.move(TEST_PWM, TEST_PWM);
  unsigned long start = millis();
  while (millis() - start < RUN_MS) {
    // optional: leftEncoder.updateSpeed(); rightEncoder.updateSpeed();
    delay(10);
  }

  // Stop
  motors.stop();
  delay(200);

  // Read ticks
  int64_t leftTicks  = leftEncoder.getCount();
  int64_t rightTicks = rightEncoder.getCount();
  int64_t avgTicks   = (leftTicks + rightTicks) / 2;

  Serial.println("=== Encoder test result ===");
  Serial.print("Left ticks: ");
  Serial.println(leftTicks);
  Serial.print("Right ticks: ");
  Serial.println(rightTicks);
  Serial.print("Average ticks: ");
  Serial.println(avgTicks);
  Serial.println();
  Serial.println("Measure the distance (meters) with your meter, then:");
  Serial.println("ticks_per_meter = Average ticks / distance_meters");
}

void loop() {
  // empty
}