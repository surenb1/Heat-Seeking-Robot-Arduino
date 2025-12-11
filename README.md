# Heat-Seeking-Robot-Arduino

## Introduction
This repository contains all the information needed for the heat-seeking robot project that was completed. It includes the source code and describes what pieces of hardware are required for the project. In this project, our goal was to create a robot that scans for and identifies a heat source which it would then lock onto and follow. If the robot should lose track of the heat source, the robot will stop and return to scanning for a new heat source.

## Results and Video Demonstration Links
- Demo 1: https://www.youtube.com/shorts/HG5WJz96h4U
- Demo 2: https://www.youtube.com/shorts/gSStDdBRn9U
- Demo 3: https://www.youtube.com/shorts/vOCAnLLsdhI

## Background and Methodology
The main embedded systems and robotics concepts applied in this project are
- I2C
- PWM
- GPIO
- Sensor Fusion
- Decision-Making Algorithms
- Interfacing with Peripherals
- Interfacing with Hardware

Our project goals were achieved by using the Adafruit AMG8833 IR Thermal Camera array to sense heat, two SG-90 servo motors (one to control left/right panning of the IR camera and one to control up/down tilt), two stepper motors to allow the tank chassis to move, a motor driver
to supply the high current required to both the stepper motors for the tank tracks (this motor driver also served as a 5V power supply to both servo motors and as a common ground between all components including the Arduino), finally, an Elegoo UNO R3 was used as the MCU (same as Arduino UNO R3).

## Functional Block Diagram
<img width="1338" height="911" alt="image" src="https://github.com/user-attachments/assets/d11d72e8-c896-4f0f-a95a-2e0fddd46d53" />

## Table of Components Used
<img width="718" height="629" alt="image" src="https://github.com/user-attachments/assets/84360c04-67da-48d9-a85f-8bc3012c6936" />

## Pinouts Used

### SG-90 Servo Pinout (Tilt Servo)
<img width="1738" height="872" alt="image" src="https://github.com/user-attachments/assets/8c96eeb5-9cd2-46ee-a4f8-af23d32c0450" />

### SG-90 Servo Pinout (Pan Servo)
<img width="1725" height="884" alt="image" src="https://github.com/user-attachments/assets/75b56180-a01a-4842-9c9d-6da410761ac7" />

### AMG8833 IR Thermal Camera Array Pinout
<img width="1652" height="866" alt="image" src="https://github.com/user-attachments/assets/dbe4da24-c726-41d1-8531-b6c51c667c29" />

### L298N Motor Driver Pinout
<img width="2433" height="1407" alt="image" src="https://github.com/user-attachments/assets/cae1c6b0-e87c-4fdf-9ffb-b4c72656850c" />






