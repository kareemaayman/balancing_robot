# Self-Balancing Robot — Embedded System (ESP32)

A two-wheeled self-balancing robot using real-time feedback control (PID) implemented on ESP32. The system maintains stability by continuously correcting tilt angle using IMU data.

## ⚙️ System Overview
The robot behaves like an inverted pendulum system. It uses feedback from an MPU6050 IMU to estimate tilt angle, then applies a PID controller to drive DC motors and maintain balance.

Main loop:
1. Read sensor data (accelerometer + gyroscope)
2. Estimate angle using MPU6050 DMP
3. Compute control output (PID)
4. Drive motors accordingly

## 🚀 Key Features
- Real-time self-balancing using PID control
- MPU6050 DMP-based orientation estimation
- Dual PID loops (angle + speed)
- Encoder-based feedback for motion control
- Modular and extensible software architecture

## 🧩 Components
- ESP32 microcontroller
- MPU6050 (Accelerometer + Gyroscope)
- Motor Driver (L298N)
- DC Gear Motors with Encoders
- Battery Pack
- Chassis + Wheels

## 🎯 Control Strategy
A PID controller is used to stabilize the robot:

- Proportional (P): reacts to current tilt
- Integral (I): eliminates steady-state error
- Derivative (D): predicts motion and reduces overshoot

The controller output adjusts motor speed and direction to keep the robot upright.

PID tuning was performed experimentally to achieve stable and responsive behavior.

## 📡 Sensor Processing
The MPU6050 Digital Motion Processor (DMP) was used to obtain a stable and reliable angle estimation.

- Utilized onboard DMP to fuse accelerometer and gyroscope data
- Reduced noise and drift without implementing external filtering algorithms
- Provided direct orientation (yaw/tilt) measurements for control

This significantly simplified processing on the ESP32 while maintaining reliable real-time performance.

## 🧪 Testing & Validation
The system was developed and validated incrementally through modular testing:

1. **IMU (DMP) Testing**
   - Verified accurate angle measurement from MPU6050
   - Ensured stable and noise-free orientation output

2. **Motor Driver Testing**
   - Validated motor movement in all directions (forward, backward, turning)
   - Checked responsiveness to control signals

3. **Encoder Testing**
   - Measured ticks per meter to calibrate motion
   - Verified speed estimation (ticks/sec)
   - Enabled feedback for distance and velocity control

4. **Angle PID Tuning**
   - Tuned P, I, D gains experimentally
   - Achieved stable balancing with minimal oscillations

5. **Speed PID Tuning**
   - Used encoder feedback for closed-loop speed control
   - Improved motion smoothness and responsiveness

This staged approach ensured each subsystem functioned correctly before full system integration.

## 📁 Repository Structure

```text
ESP32/
│
├── SelfBalancingRobot/
│   ├── main code + libraries
│
├── test/
│   ├── IMU test
│   ├── motor test
│   ├── encoder test
│   ├── PID tuning sketches
│
├── doc/
│   ├── documentation reports and videos
```
- `SelfBalancingRobot/` → Main integrated system
- `test/` → Individual subsystem testing and validation sketches

## 🧠 Software Architecture

The system is modularly designed for clarity, scalability, and testing.

### Core Modules

- **configuration.h**
  - Centralized constants and parameters shared across the system

- **Encoder (encoder.h / encoder.cpp)**
  - Reads wheel encoder data
  - Outputs speed (ticks/sec) and distance

- **IMU (imu.h / imu.cpp)**
  - Interfaces with MPU6050 using DMP
  - Provides yaw/tilt angle for control

- **Motor Driver (motor_driver.h / .cpp)**
  - Controls motor direction and speed
  - Supports forward, backward, and turning motions

- **Yaw PID**
  - Stabilizes robot angle (main balancing loop)

- **Speed PID**
  - Controls translational motion using encoder feedback

- **robot.cpp**
  - High-level control layer
  - Provides functions such as:
    - Move a predefined distance
    - Rotate a predefined angle
    - Move using forward/turn commands
    
## 🤖 Control Interface & Extensibility

The robot supports high-level motion commands:

- Forward command (linear motion)
- Turn command (angular motion)

This abstraction enables integration with:
- Joystick control
- Computer vision systems
- ROS-based navigation

For example, vision-based tracking (e.g., following a ball) can be implemented by converting camera output into forward/turn commands.

## ⚠️ Challenges
- Sensor noise and drift affecting angle estimation
- PID tuning instability (oscillations vs responsiveness trade-off)
- Motor dead zone and response delay
- Power supply variations impacting control stability

## 🚀 Future Improvements
- Integration with ROS for autonomous navigation
- Vision-based object tracking (e.g., ball following)
- Advanced control techniques (LQR / adaptive control)
- Improved mechanical stability and weight distribution
- Wireless control via WiFi/Bluetooth
