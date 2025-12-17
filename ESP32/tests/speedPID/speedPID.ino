#include <Arduino.h>
#include "EncoderReader.h"
#include "MotorDriver.h"
#include "SpeedPID.h"

MotorDriver motors(26, 27, 25, 32, 23, 33);
EncoderReader encL(34, 35);
EncoderReader encR(39, 36);

SpeedPID speedPIDL("Left", 0.5, 0.2, 0.1);
SpeedPID speedPIDR("Right", 0.5, 0.2, 0.1);

double targetSpeed = 50.0;  // ticks/sec
unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=== SPEED PID TUNING ===");
  Serial.println("Commands: TARGET <speed>, KP <val>, KI <val>, KD <val>");
  Serial.println("Example: TARGET 100");

  motors.initialize();
  encL.initialize();
  encR.initialize();

  speedPIDL.initialize();
  speedPIDR.initialize();
}

void loop() {
  handleSerial();

  encL.updateSpeed();
  encR.updateSpeed();

  double speedL = encL.getSpeedTicksPerSec();
  double speedR = encR.getSpeedTicksPerSec();

  speedPIDL.setSetpoint(targetSpeed);
  speedPIDL.setInput(speedL);
  speedPIDL.update();

  speedPIDR.setSetpoint(targetSpeed);
  speedPIDR.setInput(speedR);
  speedPIDR.update();

  int pwmL = (int)speedPIDL.getOutput();
  int pwmR = (int)speedPIDR.getOutput();

  motors.move(pwmL, pwmR);

  unsigned long now = millis();
  if (now - lastPrint >= 100) {
    lastPrint = now;
    Serial.print("Target: ");
    Serial.print(targetSpeed, 0);
    Serial.print(" | Left: ");
    Serial.print(speedL, 1);
    Serial.print(" ticks/s | Right: ");
    Serial.print(speedR, 1);
    Serial.print(" ticks/s | L_PWM: ");
    Serial.print(pwmL);
    Serial.print(" R_PWM: ");
    Serial.println(pwmR);
  }
}

void handleSerial() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toUpperCase();

  if (cmd.startsWith("TARGET ")) {
    targetSpeed = cmd.substring(7).toFloat();
    Serial.print("✓ Target = ");
    Serial.println(targetSpeed, 1);
    return;
  }

  if (cmd.startsWith("KP ")) {
    double kp = cmd.substring(3).toFloat();
    speedPIDL.setGains(kp, 0.2, 0.1);
    speedPIDR.setGains(kp, 0.2, 0.1);
    Serial.print("✓ Kp = ");
    Serial.println(kp, 2);
    return;
  }

  if (cmd.startsWith("KI ")) {
    double ki = cmd.substring(3).toFloat();
    speedPIDL.setGains(0.5, ki, 0.1);
    speedPIDR.setGains(0.5, ki, 0.1);
    Serial.print("✓ Ki = ");
    Serial.println(ki, 2);
    return;
  }

  if (cmd.startsWith("KD ")) {
    double kd = cmd.substring(3).toFloat();
    speedPIDL.setGains(0.5, 0.2, kd);
    speedPIDR.setGains(0.5, 0.2, kd);
    Serial.print("✓ Kd = ");
    Serial.println(kd, 2);
    return;
  }

  if (cmd == "STOP") {
    motors.move(0, 0);
    Serial.println("✓ Stopped");
    return;
  }
  Serial.println("? Unknown. Try: TARGET <speed>, KP <val>, KI <val>, KD <val>, STOP");
}
