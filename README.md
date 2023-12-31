# WRO Team: Rickshaw - _Anything for rickshaw!_

![front-stylish](https://github.com/mdgaziur/wro-team-rickshaw/assets/40351881/6d69a551-bd12-4ffe-ac71-d60a8d729188)
*Rickshaw in it's full glory*

## Table of Contents

- [Introduction](#introduction)
- [Repository Structure](#repository-structure)
- [Hardware Components](#hardware-components)
- [Software](#software)
- [Design decisions](#design-decisions)
- [Hardware Setup](#hardware-setup)
- [Software Setup](#software-setup)
- [The Journey](#the-journey)

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

- Raspberry PI
- PI Camera
- Arduino Mega
- Buck Converter (x2)
- Breadboard
- 11.1V 3S 1500mAh LiPo Battery
- HC-SR04 Sonar Sensor (x3)
- DC Motor (x3)
- L293D Motor Driver
- Switch
- Push Button

The PI and the motor driver(and the motors) are powered by the LiPo battery. As we're connecting the Arduino
to the PI for serial communication, it gets the necessary power from the PI. The PI and the sonar sensors get
powered by two separate buck converters for the most efficient result.

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

This part of the software controls the vehicle. To do this, it works with inputs from 3 ultrasonic sensors and
from the Raspberry PI. The Arduino follows an algorithm to control the vehicle.

The following algorithm is followed during the open challenge round:

1. Check if the vehicle is closer to the front wall than it is supposed to. Else, jump to 4.
2. If yes, check if the left wall is closer or the right wall.
3. If left wall is closer, turn right. Otherwise, turn left.
4. Check if the left wall or the right wall is closer.
5. Depending on that, go left or right slightly to make sure that the vehicle is centered.

Due to some hardware caused inconsistencies, there are some extra checks to make sure that the vehicle less
affected.

The following algorithm is followed during the obstacle challenge round:

1. Check if the vehicle is closer to the front wall than it is supposed to. Else, jump to 4.
2. If yes, check if the left wall is closer or the right wall.
3. If left wall is closer, turn right. Otherwise, turn left.
4. Move slightly to left or right depending on the detected object's color if any is detected. Otherwise jump to 5.
5. Check if the left wall or the right wall is closer.
6. Depending on that, go left or right slightly to make sure that the vehicle is centered.

## Design Decisions

Certain design decisions have been taken to make sure the vehicle works in the most optimal way and based on the availability
of parts in our region.

### The usage of Raspberry PI for object detection

We've chosen this component because of it's extensibility and customizability. Raspberry PI is powered by Linux. So, we can tweak
it as much as we want to match the optimal settings for operating the vehicle. Such tweakings include(but not limited to): console
mode for reducing CPU load during object detection, adding necessary softwares/libraries for image recognitions.

This SBC(aka. Single Board Computer) can be used to directly do serial communication with the Arduino Mega to send object detection data through USB connection. 
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

### The usage of two motors to drive the vehicle forwards or backwards

The motor in the RC car's chassis can't generate enough torque to move the vehicle forward or backward. To remedy this, we've come up with a
design where two mechanically conncted DC motors. These generate enough torque to move the vehicle.

## Hardware setup

Firstly, solder jumper wires with appropriate colors to the buck converters. Female jumper wires are preferred for OUT+ and OUT- and male
jumper wires are preferred for IN+ and IN-.

Then, Make sure you have a decently sized RC car chassis. Put the motor driver, the buck converters, the battery, the ultrasonic sensors
and a voltage rail on the bottom chassis. You may want to cut out the AAA battery compartment out of the chassis to save space. Then put 4 hex 
extenders or anything tall enough on the four holes where screws were located previous. These are located on the 4 corners of the chassis.

For the second "floor", remove everything from the chassis first. Cut out the AAA battery compartment for saving space. Make a hole on the chassis. This is 
where we'll route all the cables from the bottom chassis to the top chassis. Put the Raspberry PI, Arduino Mega, PI Camera, and a voltage
rail there. You may refer to the vehicle pictures for help.

Before putting the top chassis on top of the extenders, connect everything based on the
schematics using jumper wires but do not connect the PI to the buck converter yet. Connect the bottom voltage rail to the battery and check whether
the lights of the buck converter and the motor driver are turned on or not. If they're turned on, everything's good. If not, immediately disconnect
the battery and check every connection. The most common mistake is wrong polarity. This can be checked by checking whether any wire has become hot or
not.

If everything's ok, connect the battery and then connect your multimeter to the OUT+ and OUT- pads of the buck converter. Slowly turn the screw like
thing on the blue part of the buck converter to adjust the output voltage. You should stop when your multimeter reads 5V. Now connect the buck converter
to the Raspberry PI and put the top chassis on top of the bottom chassis. Repeat the same for the buck converter for the sonar sensors.

## Software setup

We'll start by uploading the second module of our program to the Arduino Mega which controls the vehicle. Follow the following steps to upload the
program:

- Download and install Arduino IDE (if you already don't have it installed)
- Open `src/module_2_open_challenge/module_2_open_challenge.ino` in the IDE
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

# The Journey

The journey to building the vehicle have been "rough", to say the least. We've faced extreme amounts of friction due to issues such as low availability of hardware, lack of certain services like 3D printing, etc.

The journey began with two of the teammates(MD Gaziur Rahman Noor and Syed Al Ibrahim) meeting each other through the help of the coach. We've started to plan out how we might make the final product. For an extra hand, Noor added his fellow classmate and close friend. Despite his enthusiasm and active willingness to cooperate, he had to leave the team due to extreme academic pressure. So, another friend of Noor, Adil was added. 

Initially it was planned that we'd do a 3D printing of our custom chassis. However, the lack of 3D printing service forced us to ditch that idea. This resulted in a huge amount of time being wasted. Fast forward to around 14 days before the contest, we've decided to make use of RV car's chassis. 

But then, we've got struck with another issue. The steering system of that chassis wouldn't work properly at all. Because of that, we had to get another one and design our vehicle once again. Hopefully that's the first and last design, right? right? 🥲

We kept hitting issues after issues and we couldn't solve them in time efficient way due to lack of availability of certain components. Our Arduino Mega had an already blown up voltage regulator so VIN wouldn't work. That was a massive stab in the back. Couple that with the fact that the PI would restart itself randomly, it was a truly **rough** joirney. But at least we've been able to make something, get new experiences and have nice memories :^)
