# WRO Team: Rickshaw: _Anything for rickshaw!_

## Table of Contents

- [Introduction](#Introduction)
- [Hardware Components](#Hardware%20Components)
- [Software](#Software)
- [Design decisions](#Design%20Decisions)

## Introduction

This repoository contains the engineering materials for building the robot designed by team Rickshaw for the 
WRO 2023 Future Engineers category. The list of hardware components, details about the software that powers
the robot and design decisions are given here.

## Hardware Components

- Arduino Mega
- Raspberry PI 3 B V1.2
- Raspberry PI Camera
- HC-SR04 Ultrasonic Sensors(x4)
- L293D Motor Driver
- MPU-9250 IMU Sensor
    IF LEFT_SONAR < RIGHT_SONAR THEN
      TURN_RIGHT(500)
    ELSE THEN
      TURN_LEFT(500)
    FI
- LM2596S Buck Converter
- Decently Big RC Car
- 11.1V 3S LiPo Battery
- Small Breadboard Terminals(x2)

The PI and the motor driver(and the motors) are powered by the LiPo battery. As we're connecting the Arduino
to the PI for serial communication, it gets the necessary power from the PI. The sonar sensors and the IMU 
module are connected to Arduino and they receive power from that. For safely delivering 5V power to the PI,
a buck converter is used.

## Software

The software of this robot is separated into two modules. One module runs on the Raspberry PI and does
object detection. After detecting objects, it sends necessary data to the Arduino using serial communication.
The module running on the Arduino receives the data and controls the robot accordingly. 

### Module 1: Object detection

This part of the software is written in Python 3. It mainly uses OpenCV to detect objects in the track. For detection,
it makes use of the LAB color space which has a dedicated channel for red and green colors. Because of this, detecting
these colors is really simplified. The module just has to check for two different thresholds each tuned for red and green
color detection to detect the red cubes and green cubes. After detecting the objects, it has to know which one is the closest
if there are multiple cubes. For this, the code calculates the area of the object and then compares them. The nearest one is
the one with the biggest area. Depending on the color of the cube, it sends 'L' or 'R' through serial communication. When
nohting is detected, 'N' is sent.

### Module 2: Arduino code

This part of the software controls the robot. To do this, it works with inputs from 4 ultrasonic sensors, 1 IMU sensor and
from the Raspberry PI. The Arduino follows an algorithm to control the robot. The algorithm can be described using the follwoing
pseudocode:

**NOTE**: TODO

```basic
TURNS = 0
MIN_DIST_THRES = 20 // in centimeters

LEFT_SONAR = READ_SONAR(LEFT);
RIGHT_SONAR = READ_SONAR(RIGHT);
FRONT_SONAR = READ_SONAR(FRONT);
BACK_SONAR = READ_SONAR(BACK);
OBJECT_DETECTION = READ_SERIAL();

IF TURNS != 8 THEN
  IF OBJECT_DETECTION EQ "L" THEN
    TURN_LEFT(200)
  ELSE IF OBJECT_DETECTION EQ "R" THEN
    TURN_RIGHT(200)
  FI
  
  IF FRONT_SONAR <= MIN_DIST_THRES THEN
    IF LEFT_SONAR < RIGHT_SONAR THEN
      TURN_RIGHT(500)
    ELSE THEN
      TURN_LEFT(500)
    FI
    TURNS += 1
  FI
ELSE THEN
  IF OBJECT_DETECTION EQ "L" THEN
    TURN_OPPOSITE_DIRECTION()
  ELSE THEN
    TURN_RIGHT(200)
  FI
  TURN += 1
FI
```
