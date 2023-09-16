# WRO Team: Rickshaw - _Anything for rickshaw!_

## Table of Contents

- [Introduction](#introduction)
- [Repository Structure](#repository-structure)
- [Hardware Components](#hardware-components)
- [Software](#software)
- [Design decisions](#design-decisions)
- [Hardware Setup](#hardware-setup)
- [Software Setup](#software-setup)

## Introduction

This repoository contains the engineering materials for building the vehicle designed by team Rickshaw for the 
WRO 2023 Future Engineers category. The list of hardware components, details about the software that powers
the vehicle and design decisions are given here.

### Naming

The naming of the team is quite random. Our vehicle is called "Rickshaw"(ironically). And the motto _"Anything
for rickshaw!"_ signifies that we can do anything for the "Rickshaw" so that it works the best.

### The Team

The team consists of 3 very passionate students who are aiming to change Bangladesh through the next technological
revolution. This team is quite lucky to have a really helpful coach who's always coming up with ways to make the
best performing vehicle.

### Team Members
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
- Decently Big RC Car (x2)
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
CONST ALIGNMENT_DELAY = 100ms
CONST LR_ERROR_MARGIN = 1cm
CONST TURN_SPEED = 155
CONST NORMAL_SPEED = 200

FUNC TURN_OPPOSITE()
    SET_SPEED(TURN_SPEED)
    LAST_YAW = READ_YAW()

    WHILE ABS(READ_YAW() - LAST_YAW) < 180 DO
        WHILE READ_SONAR(BACK) > MIN_DIST_THRES DO
            GO_BACK()
            TURN_LEFT_NO_DELAY()
        DONE
        STOP_MOTOR()

        WHILE READ_SONAR(FRONT) > MIN_DIST_THRES DO
            GO_FORWARD()
            TURN_RIGHT_NO_DELAY()
        DONE
        STOP_MOTOR()
    DONE

    STOP_TURN()
    GO_FORWARD()
    SET_SPEED(NORMAL_SPEED)
END

FUNC LOOP()
    IF TURNS == 8 THEN
        LET OBJECT_DETECTION = READ_SERIAL()

        IF OBJECT_DETECTION == "R" THEN
            TURN_OPPOSITE()
        DONE

        TURNS += 1
    ELSE
        LET LEFT_SONAR = READ_SONAR(LEFT)
        LET RIGHT_SONAR = READ_SONAR(RIGHT)
        LET FRONT_SONAR = READ_SONAR(FRONT)
        LET BACK_SONAR = READ_SONAR(BACK)
        LET OBJECT_DETECTION = READ_SERIAL()

        // TURNING
        IF FRONT_SONAR < MIN_DIST_THRES THEN
            SET_SPEED(TURN_SPEED)
            IF LEFT_SONAR < MIN_DIST_THRES OR LEFT_SONAR < RIGHT_SONAR THEN
                TURN_RIGHT(TURN_DELAY_OPEN, TURN_ANGLE)
            ELSE
                TURN_LEFT(TURN_DELAY_OPEN, TURN_ANGLE)
            DONE

            TURNS += 1
            SET_SPEED(NORMAL_SPEED)
        DONE

        LET LEFT_SONAR = READ_SONAR(LEFT)
        LET RIGHT_SONAR = READ_SONAR(RIGHT)
        LET FRONT_SONAR = READ_SONAR(FRONT)

        // LANE CHANGING
        IF OBJECT_DETECTION == "L" THEN
            TURN_LEFT(TURN_DELAY_OBSTACLE)
        ELSE IF OBJECT_DETECTION == "R" THEN
            TURN_RIGHT(TURN_DELAY_OBSTACLE)
        DONE

        // CENTER ALIGNMENT
        LET LEFT_SONAR = READ_SONAR(LEFT)
        LET RIGHT_SONAR = READ_SONAR(RIGHT)
        WHILE ABS(LEFT_SONAR - RIGHT_SONAR) > LR_ERROR_MARGIN DO
            LEFT_SONAR = READ_SONAR(LEFT)
            RIGHT_SONAR = READ_SONAR(RIGHT)
            DIFF = LEFT_SONAR - RIGHT_SONAR

            IF DIFF < 0 THEN
                TURN_RIGHT(ALIGNMENT_DELAY)
            ELSE IF DIFF > 1 THEN
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
car as the chassis allowed us to get around the issue of not being able to do 3D printing or using CNC to build a custom chassis. This has
also greatly simplified the development process.

### The usage of an IMU sensor

The gyroscope inside the IMU sensor allows us to find out what's the amount of angle the vehicle has turned. This helps us to make proper turns
around corners. Also, it helps us to turn to the opposite side when necessary.

## Hardware setup

Firstly, solder jumper wires with appropriate colors to the buck converter. Female jumper wires are preferred for OUT+ and OUT- and male
jumper wires are preferred for IN+ and IN-.

Then, Make sure you have a decently sized RC car chassis. Put the motor driver, the buck converter, the battery, the ultrasonic sensors
and a voltage rail on the bottom chassis. You may want to cut out the AAA battery compartment out of the chassis to save space. Then put 4 hex 
extenders or anything tall enough on the four holes where screws were located previous. These are located on the 4 corners of the chassis. For the
second "floor", remove everything from the chassis first. Cut out the AAA battery compartment for saving space. Make a hole on the chassis. This is 
where we'll route all the cables from the bottom chassis to the top chassis. Put the Raspberry PI, Arduino Mega, PI Camera, IMU sensor, and a voltage
rail there. You may refer to the vehicle pictures for help. Before putting the top chassis on top of the extenders, connect everything based on the
schematics using jumper wires but do not connect the PI to the buck converter yet. Connect the bottom voltage rail to the battery and check whether
the lights of the buck converter and the motor driver are turned on or not. If they're turned on, everything's good. If not, immediately disconnect
the battery and check every connection. The most common mistake is wrong polarity. This can be checked by checking whether any wire has become hot or
not. If everything's ok, connect the battery and then connect your multimeter to the OUT+ and OUT- pads of the buck converter. Slowly turn the screw like
thing on the blue part of the buck converter to adjust the output voltage. You should stop when your multimeter reads 5V. Now connect the buck converter
to the Raspberry PI and put the top chassis on top of the bottom chassis. 

## Software setup

We'll start by uploading the second module of our program to the Arduino Mega which controls the vehicle. Follow the following steps to upload the
program:

- Download and install Arduino IDE (if you already don't have it installed)
- Open `src/module_2/module_2.ino` in the IDE
- Connect your Arduino Mega to your computer. Depending on your operating system, you may have to follow additional steps.
- Press the button with "=>" icon located on top left to upload the program to your Arduino Mega.

Now, let's set up our Raspberry PI for the object detection code. First install Raspbian OS onto a memory card. Follow
[this tutorial](https://www.raspberrypi.com/documentation/computers/getting-started.html) for more info. You should
not install the full version of Raspbian OS as it's bloated with softwares that aren't really necessary for our job.
The base version of the OS which is 0.8GB in size is recommended. You also must install the `32-bit` version of this 
OS because the PI camera doesn't work on 64-bit Raspbian OS.

After installing Raspbian OS, we need to enable `Legacy Camera`. First, connect a monitor with an HDMI cable to your
Raspberry PI. Now, boot up your Raspberry PI by connecting it to any mobile phone charger. After it boots up, follow
along with the setup screen. After finishing that, open up terminal and run `sudo raspi-config`. Then, using arrow keys,
head to `Interfaces` option. Now go to `Legacy Camera` and enable that. Finally go back to main menu and navigate to `Finish`
and press enter. When it asks for rebooting, press `Yes`. 

After restarting, run the following command to install the necessary softwares for image recognition:

```sh
sudo apt update && sudo apt install python-opencv && pip3 install imutils "picamera[array]" pyserial
```

Now, connect your camera to the Raspberry PI. After that, clone this repository into your PI and run the code inside `src/module_1/module_1.py` using the following
command:

```sh
python src/module_1/module_1.py
```

If everything's okay, you should see `No detection` on the terminal. Hold a green or a red object for testing. Now exit the program by pressing Ctrl+C.

Let's add the script to startup so that we don't have to start it manually all the time. Run the following command first:

```sh
sudo nano /etc/rc.local
```

Add the following line before `exit 0` (Assuming you cloned the repository on the home folder and the username is `pi`):

```sh
python /home/pi/wro-team-rickshaw/src/module_1/module_1.py
```

And you're done with software setup! Now connect the Arduino to the Raspberry PI and you're ready to go!
