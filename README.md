# InitialD Line Following Robot

## Overview

Welcome to the InitialD Line Following Robot project! This repository contains the Arduino code and documentation for a line-following robot designed to participate in competitions. The goal of the project is to develop a robot that can follow a designated line on the competition track and complete the course in the fastest time possible using standard line-following mathematical algorithms.

**Special Note:** This project won first place in our school competition, demonstrating excellent performance in hardware and software integration.

## Features

- **Line Detection:** Utilizes sensors to detect and follow a designated line on the track.
- **Speed Optimization:** Implements algorithms to maximize speed while maintaining accurate line tracking.
- **Obstacle Handling:** Basic obstacle detection and avoidance capabilities.
- **Modular Code:** Easily adjustable parameters for tuning performance.

## Hardware Requirements

To replicate or build upon this project, you will need the following hardware components:

- Arduino Uno or compatible microcontroller
- Line sensor array (e.g., QTR-8RC reflectance sensor array)
- Motor driver (e.g., L298N)
- DC motors with wheels
- Chassis for the robot
- Power supply (battery pack)
- Connecting wires and breadboard

## Software Requirements

- Arduino IDE
- Standard Arduino libraries
- Custom libraries for sensor and motor control (included in this repository)

## Getting Started

### Clone the Repository

```sh
git clone https://github.com/TheBlankness/initialD.git
cd initialD
```

### Setup the Arduino IDE

1. Install the [Arduino IDE](https://www.arduino.cc/en/Main/Software).
2. Open the `initialD.ino` file in the Arduino IDE.

### Hardware Assembly

1. **Sensor Array:** Connect the line sensor array to the Arduino following the sensor's datasheet instructions.
2. **Motors and Motor Driver:** Connect the motors to the motor driver and then connect the motor driver to the Arduino.
3. **Power Supply:** Ensure that the Arduino and motor driver have a stable power supply.
4. **Chassis Assembly:** Assemble all components onto the robot chassis.

### Upload the Code

1. Connect your Arduino to your computer using a USB cable.
2. Select the correct board and port from the Arduino IDE (`Tools` > `Board` and `Tools` > `Port`).
3. Upload the code to the Arduino (`Sketch` > `Upload`).

## Tuning the Algorithm

The robot's performance can be significantly improved by fine-tuning the following parameters in the code:

- **Proportional (P) Gain:** Adjusts the robot's response to line position error.
- **Integral (I) Gain:** Adjusts the robot's response to accumulated error over time.
- **Derivative (D) Gain:** Adjusts the robot's response to the rate of change of the error.

These parameters can be found in the `initialD.ino` file under the PID control section.

## Contributions

Contributions to this project are welcome. If you have any ideas for improvements or new features, feel free to fork the repository and submit a pull request.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.

## Acknowledgments

- Special thanks to our school and mentors for their support and guidance.
- Inspired by various online resources and the Arduino community.

## Contact

For any questions or further information, please contact us at [iddinishak@gmail.com](mailto:iddinishak@gmail.com).

---

Enjoy building and racing your line-following robot!

---
