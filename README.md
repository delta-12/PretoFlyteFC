# PretoFlyteFC

Simple quadcopter flight controller based on an ESP32 and LSM9DS1

## Table of Contents

<!-- TODO re-organize table of contents (repo org and build instr after design overview?) -->

- [Design Overview](#design-overview)
- [Justification](#justification)
- [Preliminary Design Verification](#preliminary-design-verification)
- [Design Implementation](#design-implementation)
- [Design Testing](#design-testing)
- [Summary, Conclusions, and Future Work](#summary-conclusions-and-future-work)
- [Repository Organization](#repository-organization)
- [Build Instructions](#build-instructions)

## Design Overview

<!-- In this section, you should provide a detailed description of your design.

- [x] Start with a high-level description of your design and its purpose.
- [ ] Discuss the original design concepts that you considered (at a high level) and then your
      final design. This document is not only a description of the technical aspects of your
      design, but it is also a digest of your design process.
- [ ] Clearly explain how your project expands and builds on previous works (i.e. justifications
      for a 6-week project)
- [ ] Don’t forget to cite your references and include images/schematics. -->

PretoFlyteFC is a simple flight controller for a small to medium-sized quadcopter and its purpose is to
stabilize roll and pitch. It primarily consists of an ESP32 microcontroller and an 9-axis LSM9DS1 inertial
measurement unit (IMU), which is equipped an accelerometer, gyroscope, and magnetometer. Generally, human
operators cannot react fast enough to even small perturbations in a quadcopter's flight, thus necessitating
additional control systems in the form of a flight controller to achieve stable flight. At a high level,
PretoFlyteFC works by reading in data from the accelerometer and gyroscope, filtering the signals,
inputting the filtered data and commanded angular set points into a PID (Proportional, Integral, Derivative)
contrl loop, and outputting roll and pitch signals that can then be used to control motor speed to achieve
the desired set point.

The original design for PretoFlyteFC was meant to control roll and pitch in addition to roll rate, pitch
rate, and yaw rate using cascaded PID for both angular and angular rate setpoints as well as implement
motor mixing and direct communication with an electronic speed controller (ESC) for the motors. However,
due to time constraints, broken parts, and difficulties communicating with the ESC, the PretoFlyteFC was
narrowed in scope to only cover control of the roll and pitch angles.

_Original design_![Quadcopter full PID loop](assets/high_level_control.drawio.png)

_Revised design_![Quadcopter revised PID loop](assets/high_level_control_revised.drawio.png)

The quadcopter used to test and validate PretoFlyteFC was built from the SpeedyFPV 220 Racing Drone Kit
purchased from [Amazon.com](https://a.co/d/abwnG7r). The kit contains nearly all the essentials needed to
build an FPV (first person view) racing quadcopter, including a frame, motors, props, ESC, and even a flight
controller. The only part not included is a battery.

Inspiration and guidance for this project came from the _How to Write your own Flight Controller Software_ articles by David Such and the Carbon Aeronautics YouTube video series about how to build an drone based on Arduino.

_How to Write your own Flight Controller Software_: [https://reefwing.medium.com/how-to-write-your-own-flight-controller-software-part-1-ac08b6ecc01e](https://reefwing.medium.com/how-to-write-your-own-flight-controller-software-part-1-ac08b6ecc01e)

Carbon Aeronautics YouTube channel: [https://www.youtube.com/@carbonaeronautics/videos](https://www.youtube.com/@carbonaeronautics/videos)

## Justification

<!-- TODO mention limited exposure to drones -->

This project was purely a learning experience. I undertook this project because I have always been
interested in drones and UAVs but never had the time to work on a design like this. This project is also
a good opportunity to apply some of the control theory I have been studying in my mechatronics course and
to learn about signal filtering and sensor fusion. Furthermore, as someone who also plans to specifically
study drones and control theory in future courses, this project is a good introduction prior to the start
more formal study. Lastly, I plan to pursue a carrer in robotics in which all of these skills involved in
this project are very relevant and would strengthen my resume. After some initial testing and limiting the
scope of the design, I believed this project was meet the desired goals in the given time frame.

## Preliminary Design Verification

### Spinning the Motors

Some of the first parts of the quadcopter that were tested were the motors. After soldering the motors to the
ESC, the flight controller included in the kit was connected to the ESC with a provided cable. The specific
flight controller is the F4 Noxe v3 and it runs a well-known flight controller software called [Betalfight](https://betaflight.com/).
Betaflight has a software configuration tool that is used to setup and tune flight controllers running
Betaflight. The configuration tool is run on a computer connected to a Betaflight flight controller over USB.
The Betaflight configuration tool features a menu to configure and test a quadcopter's motors, which in this
case could be used to manually spin the motors to verify the worked and had be soldered correctly to the ESC.

![Betaflight configuration tool - motors](assets/betaflight_motor_test.png)_Betaflight configuration tool - motor test_

Upon verifying the ESC and motors worked by testing them with the Betaflight configuration tool, basic test
was done to see if the ESP32 could directly communicate the ESC to control the motors. There are several
different protocols used to communicate with ESCs such as Oneshoot, Multishot and DSHOT. Raw PWM signals can
also be used. The ESC included in the kit is a 4-in-1 45A ESC running the BLHeli_S firmware, and in theory it
is capable of detecting which protocl is being used for communication on the fly without any additional input.
To test if the ESP32 could communicate with the motors, a simple Arduino program was written to output a PWM
signal on a few of the ESP32's GPIO. These GPIO were then to connected to a input pads on the ESC for each
of the four motors. A simple circuit using the LM7805 linear voltage regulator was also contructed on a
breadboard to regulate the 7-8V supplied by the ESC down to the 5V required by the ESP32. Unfortuntely, the
ESP32 was unable to control the motors by sending PWM signals to the ESC. PWM from the ESP32 running the
Arduino test program resulted in intermitent twitching motors and random spinning instead of smooth rotation.
Therefore, since the flight controller included in the kit was known to be able successfully communicate with
the ESC and smoothly spin the motors, the decision was made that the F4 Noxe v3 flight controller would act as
a gateway between the ESP32 and the ESC, with the ESP32 sending commands to the F4 Noxe which would then in turn
send commands to the ESC. To accomplish this, the ESP32 would masquerade as a radio receiver connected to the
F4 Noxe and send roll, pitch, yaw, and throttle commands to it over a serial protocol commonly used by radio
controllers called SBUS. The F4 Noxe would then use DSHOT to communicate with the ESC. It was assumed that
writing an SBUS driver for the ESP32 would also be easier than writing a driver for one of the other protocols
used to communicate directly with an ESC.

![ESP32 to F4 Noxe v3 to ESC](assets/esc_communication_overview.drawio.png)_F4 Noxe v3 flight controller acts as SBUS-to-DSHOT gateway between ESP32 and ESC_

### IMU

The LSM9DS1 was chosen for this project because I already had one in my possession and fairly easy to work with.
It has largely been discontinued, but there is still plenty of support and documentation for it online. To verify
the IMU worked and communication could be established with it, a basic test program was written in Arduino and
uploaded to an Arduino Uno. The Uno was connected over SPI to the LSM9DS1 breakout board on a breadboard, and
was able to successfully print out the values of the X, Y, Z vectors for the accelerometer and gyroscope.

### Radio Transmitter/Receiver

Likewise, I was also already in possession of an old HK-T6A V2 radio transmitter and receiver. This a six-channel,
2.4 GHz digital proportional radio control system. Documentation for this radio system is scarce, especially
given that is discontinued. These older radio controllers often use PWM or PPM to communicate. In order to
determine which protocol this particular radio uses, receiver channels were connected to an oscilliscope while
the sticks, switches, and knobs on the radio transmitter were actuated. The scope displayed PWM signals and
measured the frequency to be around 50Hz with duty cycles varying between 4% and 10% depending on the position
or state of a given input to the transmitter.

![HK-T6A V2 radio transmitter and receiver](assets/radio_transmitter_receiver.jpg)

## Design Implementation

<!-- In this section, you will explain the final design you arrived at. Include

- Include an overview of the overall system, described at a high level
- Include a listing of all the relevant subcomponents developed/used in creating the
  final software/hardware
- If there is anything else that is notable about your design, or design practices, please
  include it in this section. Discuss your design process in developing the system and
  any challenges that came about. -->

<!-- TODO describe purpose of ESC and why it's needed for BLDC motors -->
<!-- Use of included flight controller also simplified the project by implementing motor mixing and rate control -->

## Design Testing

<!-- Write a summary of testing results from your final prototype in this section.

- Explain your test plan, the procedures you use to test your design and the outcomes from
  those test.
- Include photos of the assembled prototype, and photos from any testing results
- If you had unsuccessful attempts, include write-ups of those attempts and how you remedied
  those issues.
- Make sure to document your debugging process and any challenges that came about.
- If your design is not fully functional, make sure to include write-ups on WHY it is not
  functioning
- Include references to demonstration videos that you produce (e.g. youtube or videos
  uploaded to CANVAS) showing your design functioning. Video demonstration is required to
  be in either the report, or the final presentation. -->

Testing and debugging:

<!-- - Had to re-pin the connector for the FC/ESC
- Motor re-mapping
- Use of logic analyzer
- Plotter tool
- Balance by hand
- Fly by hand for sense of gains
- Betaflight configuraton software to verify signals

- Verified that flight controller received proper SBUS commands using flight controller configuration software (SBUS driver and level shifting)
- Already had two IMU in possession, researched the difficulty for building a driver for each, ended using the older, less sophisticated one because easier to implement driver and simpler to work with
- Verified IMU driver worked with breadboard, printed out pitch and roll
- Verified the drone could be armed and props could be spun using SBUS driver
 -->

## Summary, Conclusions, and Future Work

Write a brief summary of your project and your conclusions from this assignment. Include a detailed
discussion of changes you would make to improve the design if you were to do another design iteration.

- Improve IMU driver and filtering
- Develop physical model of the system, better PID tuning, build rig for tuning
- Implement complete control loop from original design, control ESC directly
- Perf board, PCB
- Balance the quadcopter
- Higher loop rates

## Repository Organization

```
├── .github
│   └── workflows
├── assets
├── components
│   └── Sbus
|       └── include
├── main
│   └── include
└── tools
    └── cppcheck
```

## Build Instructions
