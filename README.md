# WRO Team: Rickshaw - _Anything for rickshaw!_

## Table of Contents

- [Introduction](#introduction)
- [Repository Structure](#repository-structure)
- [Hardware Components](#hardware-components)
- [Software](#software)
- [Design decisions](#design-decisions)

## Introduction

This repoository contains the engineering materials for building the vehicle designed by team Rickshaw for the 
WRO 2023 Future Engineers category. The list of hardware components, details about the software that powers
the vehicle and design decisions are given here.

### The Team

The team consists of 3 very passionate students who are aiming to change Bangladesh through the next technological
revolution. This team is quite lucky to have a really helpful coach who's always coming up with ways to make the
best performing vehicle.

- Al Mubin Khan Nabil [@nirobnabil](https://github.com/nirobnabil) (Coach)
- MD Gaziur Rahman Noor [@mdgaziur](https://github.com/mdgaziur)
- Ibrahim Al Syed [@ibrahimalsyed60](https://github.com/ibrahimalsyed60)
- Abdus Sami Mohammad Adil [@adilahmed81](https://github.com/adilahmed81)

## Repository Structure

- `schematics`: Contains breadboard circuit image, schematic and a fritzing file containing the schematic of the vehicle
- `src`: Contains the source code to both software modules
- `team-photos`: Contains one funny and one normal photo of the team
- `vehicle-photos`: Contains photos of the vehicle from top, bottom, left, right, front and back side
- `video`: Contains `video.md` containing a link to the video demonstrating the vehicle

## Hardware Components

- Arduino Mega
- Raspberry PI 3 B V1.2
- Raspberry PI Camera
- HC-SR04 Ultrasonic Sensors(x4)
- L293D Motor Driver
- MPU-9250 IMU Sensor
- LM2596S Buck Converter
- Decently Big RC Car
- 11.1V 3S LiPo Battery
- Small Breadboard Terminals(x2)

The PI and the motor driver(and the motors) are powered by the LiPo battery. As we're connecting the Arduino
to the PI for serial communication, it gets the necessary power from the PI. The sonar sensors and the IMU 
module are connected to Arduino and they receive power from that. For safely delivering 5V power to the PI,
a buck converter is used.

## Software

The software of this vehicle is separated into two modules. One module runs on the Raspberry PI and does
object detection. After detecting objects, it sends necessary data to the Arduino using serial communication.
The module running on the Arduino receives the data and controls the vehicle accordingly. 

### Module 1: Object detection

This part of the software is written in Python 3. It mainly uses OpenCV to detect objects in the track. For detection,
it makes use of the LAB color space which has a dedicated channel for red and green colors. Because of this, detecting
these colors is really simplified. The module just has to check for two different thresholds each tuned for red and green
color detection to detect the red cubes and green cubes. After detecting the objects, it has to know which one is the closest
if there are multiple cubes. For this, the code calculates the area of the object and then compares them. The nearest one is
the one with the biggest area. Depending on the color of the cube, it sends 'L' or 'R' through serial communication. When
nohting is detected, 'N' is sent.

### Module 2: Arduino code

This part of the software controls the vehicle. To do this, it works with inputs from 4 ultrasonic sensors, 1 IMU sensor and
from the Raspberry PI. The Arduino follows an algorithm to control the vehicle. The algorithm can be described using the following
pseudocode:

**TODO**: TEST THE ALGORITHM, FINE TUNE THE CONSTANTS AND FINISH THE DOCUMENTATION.

```basic
CONST TURNS = 0
CONST MIN_DIST_THRES = 20cm
CONST TURN_ANGLE = 45deg
CONST TURN_DELAY_OPEN = 500ms
CONST TURN_DELAY_OBSTACLE = 200ms
CONST ALIGNMENT_DELAY = 100
CONST LR_ERROR_MARGIN = 1cm

FUNC TURN_OPPOSITE()
    LAST_YAW = READ_YAW()

    WHILE ABS(CURRENT_YAW - LAST_YAW) < 180; DO
        WHILE READ_SONAR(BACK) > MIN_DIST_THRES; DO
            GO_BACK()
            TURN_LEFT_NO_DELAY()
        DONE
        STOP_MOTOR()

        WHILE READ_SONAR(FRONT) > MIN_DIST_THRES; DO
            GO_FORWARD()
            TURN_RIGHT_NO_DELAY()
        DONE
        STOP_MOTOR()
    DONE

    STOP_TURN()
    GO_FORWARD()
END

FUNC LOOP()
    IF TURNS == 8; THEN
        LET OBJECT_DETECTION = READ_SERIAL();

        IF OBJECT_DETECTION == "R"; THEN
            TURN_OPPOSITE()
        DONE

        TURNS += 1
    ELSE
        LET LEFT_SONAR = READ_SONAR(LEFT);
        LET RIGHT_SONAR = READ_SONAR(RIGHT);
        LET FRONT_SONAR = READ_SONAR(FRONT);
        LET BACK_SONAR = READ_SONAR(BACK);
        LET OBJECT_DETECTION = READ_SERIAL();

        // TURNING
        IF FRONT_SONAR < MIN_DIST_THRES; THEN
            IF LEFT_SONAR < MIN_DIST_THRES OR LEFT_SONAR < RIGHT_SONAR; THEN
                TURN_RIGHT(TURN_DELAY_OPEN, TURN_ANGLE)
            ELSE
                TURN_LEFT(TURN_DELAY_OPEN, TURN_ANGLE)
            DONE
        DONE

        LET LEFT_SONAR = READ_SONAR(LEFT);
        LET RIGHT_SONAR = READ_SONAR(RIGHT);
        LET FRONT_SONAR = READ_SONAR(FRONT);

        // LANE CHANGING
        IF OBJECT_DETECTION == "L"; THEN
            TURN_LEFT(TURN_DELAY_OBSTACLE)
        ELSE IF OBJECT_DETECTION == "R"; THEN
            TURN_RIGHT(TURN_DELAY_OBSTACLE)
        DONE

        // CENTER ALIGNMENT
        LET LEFT_SONAR = READ_SONAR(LEFT);
        LET RIGHT_SONAR = READ_SONAR(RIGHT);
        WHILE LEFT_SONAR - RIGHT_SONAR > LR_ERROR_MARGIN; DO
            LEFT_SONAR = READ_SONAR(LEFT);
            RIGHT_SONAR = READ_SONAR(RIGHT);
            DIFF = LEFT_SONAR - RIGHT_SONAR

            IF DIFF < 0; THEN
                TURN_RIGHT(ALIGNMENT_DELAY)
            ELSE IF DIFF > 1; THEN
                TURN_LEFT(ALIGNMENT_DELAY)
            DONE
        DONE
    DONE
END
```

## Design Decisions

Certain design decisions have been taken to make sure the vehicle works in the most optimal way and based on the availability
of parts in our region.

### The usage of Raspberry PI for object detection

We've chosen this component because of it's extensibility and customizability. Raspberry PI is powered by Linux. So, we can tweak
it as much as we want to match the optimal settings for operating the vehicle. Such tweakings include(but not limited to): console
mode for reducing CPU load during object detection, adding necessary softwares/libraries for image recognitions. This SBC(aka. Single
Board Computer) can be used to directly do serial communication with the Arduino Mega to send object detection data through USB connection. 
This allows us to avoid using complex circuitries for communication between PI and Arduino. The VNC and SSH capabilities of PI also allowed
us to easily do software development of the vehicle directly through a single interface. This reduced the amount of disconnecting and 
reconnecting stuff for debugging and fixing issues.

### Not using any camera with object detection capability

We've not chosen to bother using any camera with object detection on our vehicle. This is primarily due to such components not being available
in our region. However, the Arduino Mega is coded in such way that such components can be easily integrated with minimal changes to the code.

### The usage of Arduino Mega

Arduino Mega has just the right capability for a project like this. It has the right amount of processing power, memory and pins that make it
the best choice for this vehicle.

### The usage of an RC car as the chassis instead of a custom built one

This decision has been partly influenced by the scarcity of 3D printing in our region and partly by the ease of reproducability. Using an RC
car as the chassis allowed us to get around the issue of not being able to do 3D printing or using CNC to build a custom chassis.

### The usage of an IMU sensor

The gyroscope inside the IMU sensor allows us to find out what's the amount of angle the vehicle has turned. This helps us to make proper turns
around corners. Also, it helps us to turn to the opposite side when necessary.

