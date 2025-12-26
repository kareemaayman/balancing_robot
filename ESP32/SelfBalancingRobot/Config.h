#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================================
// CONFIGURATION LIBRARY - Central configuration for Self-Balancing Robot
// ============================================================================

namespace Config {

    // ========== MOTOR PINS ==========
    namespace Motor {
        // Left Motor
        const uint8_t LEFT_IN1 = 26;
        const uint8_t LEFT_IN2 = 27;
        const uint8_t LEFT_ENA = 25;
        
        // Right Motor
        const uint8_t RIGHT_IN3 = 32;
        const uint8_t RIGHT_IN4 = 23;
        const uint8_t RIGHT_ENB = 33;
        
        // PWM Settings
        const uint32_t PWM_FREQ = 1000;
        const uint8_t PWM_RES = 8;
        
        // Speed Limits
        const uint8_t MIN_SPEED = 60;
        const uint8_t MAX_SPEED = 255;
        
        // Correction Factors (for drift compensation)
        const double CORRECTION_LEFT_DEFAULT = 1.0;
        const double CORRECTION_RIGHT_DEFAULT = 1.0;
    }

    // ========== ENCODER PINS ==========
    namespace Encoder {
        // Left Encoder
        const uint8_t LEFT_A = 5;
        const uint8_t LEFT_B = 15;
        
        // Right Encoder
        const uint8_t RIGHT_A = 18;
        const uint8_t RIGHT_B = 19;
    }

    // ========== ANGLE PID ==========
    namespace AnglePID {
        const double KP = 20.0;
        const double KI = 0.0;
        const double KD = 0.5;
        const unsigned long SAMPLE_TIME_MS = 10;
        const double UPRIGHT_ANGLE = 90.0;
        const double K_MOVE = 2.0;  // Forward command gain
        const double FORWARD_CMD_DEFAULT = 0.0;  // Default forward command
    }

    // ========== SPEED PID ==========
    namespace SpeedPID {
        const double KP = 0.5;
        const double KI = 0.2;
        const double KD = 0.1;
    }

    // ========== YAW PID ==========
    namespace YawPID {
        const double KP = 0.5;      // Proportional gain for yaw control
        const double KI = 0.05;     // Integral gain for yaw control
        const double KD = 0.2;      // Derivative gain for yaw control
        const double SETPOINT_DEFAULT = 0.0;  // Default target yaw angle (degrees)
        const unsigned long SAMPLE_TIME_MS = 10;  // PID computation sample time
    }

    // ========== ROBOT CONTROL ==========
    namespace Robot {
        const unsigned long CONTROL_SAMPLE_TIME = 10;
        const double TURN_CMD_DEFAULT = 0.0;  // Default turn command
        const double SPEED_SCALE = 5.0;
        const double K_TURN = 20.0;
        const double TICKS_PER_METER = 1000.0;  // Encoder ticks per meter traveled (calibrate later)
    }

    // ========== IMU SETTINGS ==========
    namespace IMU {
        // I2C Address
        const uint8_t MPU_ADDR = 0x68;
    }

    // ========== SERIAL & TIMING ==========
    namespace Serial_Config {
        const uint32_t BAUD_RATE = 115200;
        const unsigned long STARTUP_DELAY = 2000;
        const unsigned long PRINT_INTERVAL = 200;
    }

    // ========== MOTOR CORRECTION FACTORS ==========
    // These are empirically determined correction factors for motor speed matching
    namespace MotorCorrection {
        const double CORRECTION_BAND_1_LOW = 80;
        const double CORRECTION_BAND_1_HIGH = 100;
        const double CORRECTION_BAND_1_FACTOR = 0.84;
        
        const double CORRECTION_BAND_2_LOW = 100;
        const double CORRECTION_BAND_2_HIGH = 150;
        const double CORRECTION_BAND_2_FACTOR = 0.9;
        
        const double CORRECTION_BAND_3_HIGH = 80;
        const double CORRECTION_BAND_3_FACTOR = 0.8;
    }

}

#endif
