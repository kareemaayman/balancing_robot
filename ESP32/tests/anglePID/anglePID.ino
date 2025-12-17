#include <Arduino.h>
#include "IMU.h"
#include "AnglePID.h"
#include "MotorDriver.h"
#include "BalanceConfig.h"

// Hardware instances
IMU imu;
AnglePID anglePID(20.0, 0.0, 0.0);  // Start with safe defaults
MotorDriver motors(26, 27, 25, 32, 23, 33);

// Tuning state
double Kp = 20.0, Ki = 0.0, Kd = 0.0;
int testPWMLimit = 120;  // Constrain output to safe range
bool motorsEnabled = false;
unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n=== ANGLE PID TUNING ===");
  Serial.println("Commands: START, STOP, Kp <val>, Ki <val>, Kd <val>, LIMIT <val>");
  Serial.println("Example: Kp 25");
  Serial.println();

  motors.initialize();
  
  if (!imu.initialize()) {
    Serial.println("ERROR: IMU failed!");
    while (1);
  }

  anglePID.initialize();
  anglePID.setGains(Kp, Ki, Kd);

  delay(500);
  Serial.println("Ready. Type START to begin tuning.");
}

void loop() {
  // Handle serial commands
  handleSerialInput();

  if (!motorsEnabled) {
    motors.move(0, 0);
    delay(10);
    return;
  }

  // Update IMU
  if (!imu.updateAngle()) {
    return;  // No new data
  }

  double angle = imu.getPitchAngle();

  // Update angle PID
  anglePID.update(angle);
  double output = anglePID.getOutput();

  // Constrain to safe testing range
  output = constrain(output, -testPWMLimit, testPWMLimit);

  // Scale to left/right motors (equal for now)
  int leftPWM = (int)output;
  int rightPWM = (int)output;

  motors.move(leftPWM, rightPWM);

  // Print debug every 100 ms
  unsigned long now = millis();
  if (now - lastPrint >= 100) {
    lastPrint = now;

    Serial.print("[");
    Serial.print(now / 1000);
    Serial.print("s] Angle: ");
    Serial.print(angle, 2);
    Serial.print("° | Error: ");
    Serial.print(angle - upright_angle, 2);
    Serial.print("° | PID Out: ");
    Serial.print(output, 1);
    Serial.print(" | PWM: L=");
    Serial.print(leftPWM);
    Serial.print(" R=");
    Serial.print(rightPWM);
    Serial.print(" | Kp=");
    Serial.print(Kp, 1);
    Serial.print(" Ki=");
    Serial.print(Ki, 1);
    Serial.print(" Kd=");
    Serial.println(Kd, 1);
  }
}

void handleSerialInput() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toUpperCase();

  // START / STOP
  if (cmd == "START") {
    motorsEnabled = true;
    motors.move(0, 0);
    delay(100);
    Serial.println("✓ Motors ENABLED - hold robot gently!");
    return;
  }

  if (cmd == "STOP") {
    motorsEnabled = false;
    motors.move(0, 0);
    Serial.println("✓ Motors DISABLED");
    return;
  }

  // Parse parameter commands: "Kp 25.5"
  if (cmd.startsWith("KP ")) {
    Kp = cmd.substring(3).toFloat();
    anglePID.setGains(Kp, Ki, Kd);
    Serial.print("✓ Kp = ");
    Serial.println(Kp, 2);
    return;
  }

  if (cmd.startsWith("KI ")) {
    Ki = cmd.substring(3).toFloat();
    anglePID.setGains(Kp, Ki, Kd);
    Serial.print("✓ Ki = ");
    Serial.println(Ki, 2);
    return;
  }

  if (cmd.startsWith("KD ")) {
    Kd = cmd.substring(3).toFloat();
    anglePID.setGains(Kp, Ki, Kd);
    Serial.print("✓ Kd = ");
    Serial.println(Kd, 2);
    return;
  }

  if (cmd.startsWith("LIMIT ")) {
    testPWMLimit = cmd.substring(6).toInt();
    Serial.print("✓ Output limit = ");
    Serial.println(testPWMLimit);
    return;
  }

  if (cmd == "STATUS") {
    Serial.println("\n=== CURRENT CONFIG ===");
    Serial.print("Motors: ");
    Serial.println(motorsEnabled ? "ON" : "OFF");
    Serial.print("Kp = ");
    Serial.println(Kp, 2);
    Serial.print("Ki = ");
    Serial.println(Ki, 2);
    Serial.print("Kd = ");
    Serial.println(Kd, 2);
    Serial.print("PWM Limit = ");
    Serial.println(testPWMLimit);
    Serial.println();
    return;
  }

  Serial.println("? Unknown command. Try: START, STOP, Kp <val>, Ki <val>, Kd <val>, LIMIT <val>, STATUS");
}
