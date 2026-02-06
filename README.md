This project implements a rocket flight control and safety system using an Arduino microcontroller, an MPU6050 Inertial Measurement Unit (IMU), Kalman filtering, PID control, and servo-based thrust vector control (TVC).
The system is designed to manage different rocket operation states such as disarmed, standby, armed, launch detection, active flight stabilization, abort, parachute deployment, and landing confirmation.

🔧 Key Features

IMU-Based Orientation Estimation

Uses the MPU6050 accelerometer and gyroscope.

Implements a Kalman filter for accurate roll and pitch angle estimation.

Complementary and gyro-only angle calculations are also included for comparison.

Thrust Vector Control (TVC)

Two servo motors control rocket orientation in X and Y axes.

PID controller stabilizes the rocket during flight.

Servo outputs are limited to safe mechanical ranges.

Launch Detection

Launch is detected automatically using acceleration thresholds.

Once detected, the system transitions from “armed” to “active TVC mode”.

Flight Safety & Abort Logic

Continuous monitoring of orientation angles.

Automatic flight abort if attitude exceeds safe limits.

Pyrotechnic servo activation for emergency deployment.

State-Based Flight Logic

Disarmed → Standby → Testing → Armed → Launch → TVC → Abort/Landing.

Visual (LED) and audio (buzzer) feedback for each state.

Post-Landing Confirmation

Detects landing based on attitude stability.

Plays a victory melody through a buzzer after successful landing.

I2C Low-Level Communication

Custom I2C read/write functions for robustness and timeout handling.

High-speed I2C (400 kHz) configuration for fast sensor updates.

🧠 Control Algorithms Used

Kalman Filter for sensor fusion (accelerometer + gyroscope).

PID Controller with:

Proportional, Integral, and Derivative terms

Derivative low-pass filtering (τ parameter)

Complementary Filter as a backup orientation estimator.

Demonstration:-
https://drive.google.com/drive/folders/1Nr3FfI15PP4-sbhMdGo8hZtSt1pCJ5GY?usp=sharing

🧩 Hardware Used

Arduino-compatible microcontroller

MPU6050 IMU

Servo motors (TVC + pyro servo)

Buzzer

LEDs

Safety and control buttons
