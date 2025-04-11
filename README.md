# ENGR290
🛸 Autonomous Hovercraft Maze Navigator
This project contains the C++ code for an autonomous hovercraft designed to navigate a maze without human input. Using a combination of ultrasonic sensors, a gyroscope (MPU6050), and a servo-based steering system, the hovercraft can detect obstacles and autonomously make navigation decisions to traverse through a maze environment.

📌 Features
🧠 Autonomous Navigation: Real-time obstacle detection and path correction

🌀 Gyroscopic Turning: Smooth, angle-based turns using yaw data

🧭 Dual-Side Sensing: Ultrasonic sensors on both sides for wall-following logic

🛠️ Servo-Based Steering: Real-time servo adjustments to correct heading

⚙️ Fan Control: PWM-based lift and thrust fan management

📋 Hardware Requirements
Microcontroller (Arduino Uno/Nano or compatible)

1x MPU6050 (Gyroscope + Accelerometer)

2x Ultrasonic sensors (HC-SR04 or similar)

1x Servo motor (HS 422)

2x Brushless or DC fans (for lift and thrust)

Power supply or battery pack

Jumper wires, breadboard, and connectors

🧠 How It Works
Initialization:

Gyroscope is calibrated at startup to determine yaw bias.

Fans are powered up for lift and forward motion.

Servo is attached to the rudder.

Navigation Loop:

Distance is continuously measured using two ultrasonic sensors.

Yaw angle is calculated from the MPU6050 gyroscope’s Z-axis.

Based on sensor readings, the hovercraft decides to turn left, right, or move straight.

Turns are controlled using yaw feedback and servo adjustments.

The system enters a cooldown period after a turn to avoid oscillations or repeated turns.

Turning:

If the right side is significantly more open than the left, the hovercraft turns right (and vice versa).

Turning uses yaw angle integration to ensure a complete 180-degree turn.

After turning, fans briefly stop and restart for smoother transitions.

🗂️ Code Structure
setup(): Initializes gyroscope, sensors, and fans.

loop(): Handles real-time sensor reading, yaw calculation, and decision-making.

turnLeft() / turnRight(): Calculates and sets servo angle based on yaw.

readDistanceFromSensors(): Returns distance in cm from ultrasonic sensors.

readGyroZValue() / calibrateGyroZ(): Reads and calibrates gyroscope for accurate yaw tracking.

recalibratePosition(): Optional fine-tuning of heading using yaw angle or sensor difference.

🧪 Tuning Parameters
Adjust these variables at the top of the code for better performance:

float c = 30; — Ultrasonic distance offset for triggering turns

float gyroBiasZaxis = 0.1; — Gyroscope bias (automatically calibrated)

int k = 1; — Sensitivity of steering angle to yaw changes

position constraints (constrain()) — Servo angle limits

🚀 Getting Started
Wire up the sensors, fans, and servo to the correct pins (see #define section).

Upload the code to your Arduino using the Arduino IDE.

Power on the hovercraft in a test maze environment.

Monitor Serial output (115200 baud) for debugging yaw angle and distance data.

⚠️ Notes
Use a stable power supply for consistent fan and servo behavior.

Ensure that the hovercraft is level during gyroscope calibration.

Serial output is commented out by default; uncomment for debugging
