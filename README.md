Self-Balancing Robot (STM32F407)
A two-wheeled self-balancing robot project based on the STM32F407VET6 microcontroller and the MPU6050 6-axis motion sensor. This project demonstrates the implementation of a single-loop PID control algorithm to achieve and maintain dynamic vertical balance.

Hardware & Technologies
MCU: STM32F407VET6 (ARM Cortex-M4).
 - Sensor: MPU6050 (I2C interface, data fusion via Complementary/Kalman filtering).
 - Actuators: DC motors with encoders for velocity feedback.
 - Control Strategy: Single-loop PID control (Angle feedback).
 - Development Environment: STM32CubeIDE.

Key Features
 - High-speed I2C communication for accurate sensor data acquisition
 - Single-loop PID implementation to stabilize the robot at the setpoint angle.
 - Robust sensor noise reduction and filtering techniques.
 - Pulse Width Modulation (PWM) motor control with driver integration.
