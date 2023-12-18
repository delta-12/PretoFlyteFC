# PretoFlyteFC

Simple quadcopter flight controller based on an ESP32 and LSM9DS1

## Table of Contents

<!-- TODO re-organize table of contents (repo org and build instr after design overview?) -->

- [Design Overview](#design-overview)
- [Justification](#justification)
- [Preliminary Design Verification](#preliminary-design-verification)
  - [Spinning the Motors](#spinning-the-motors)
  - [IMU](#imu)
  - [Radio Transmitter/Receiver](#radio-transmitterreceiver)
- [Design Implementation](#design-implementation)
  - [Control Architecture](#control-architecture)
  - [Motors and ESC](#motors-and-esc)
  - [Quadcopter Frame](#quadcopter-frame)
  - [Radio Control System](#radio-control-system)
  - [IMU](#imu-1)
  - [SBUS](#sbus)
    <!-- - [Github Actions](#github-actions) -->
    <!-- - [Bill of Materials](#bill-of-materials) -->
- [Design Testing](#design-testing)
- [Summary, Conclusions, and Future Work](#summary-conclusions-and-future-work)
- [Repository Organization](#repository-organization)
- [Build Instructions](#build-instructions)

## Design Overview

PretoFlyteFC is a simple flight controller for a small to medium-sized quadcopter and its purpose is to
stabilize roll and pitch. It primarily consists of an ESP32 microcontroller and an 9-axis LSM9DS1 inertial
measurement unit (IMU), which is equipped an accelerometer, gyroscope, and magnetometer. Generally, human
operators cannot react fast enough to even small perturbations in a quadcopter's flight, thus necessitating
additional control systems in the form of a flight controller to achieve stable flight. At a high level,
PretoFlyteFC works by reading in data from the accelerometer and gyroscope, filtering the signals,
inputting the filtered data and commanded angular set points into a PID (Proportional, Integral, Derivative)
contrl loop, and outputting roll and pitch signals that can then be used to control motor speed to achieve
the desired set point.

![Quadcopter Roll, Pitch, and Yaw](assets/quadcopter_roll_pitch_yaw.png)

_Quadcopter roll, pitch, and yaw; Source: [http://blog.rctoysky.com/?p=128](http://blog.rctoysky.com/?p=128)_

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

<!-- TODO complete system diagram -->

The microcontroller selected for this project was the ESP32 because I am already in possession of several
ESP32 development boards and am very familiar with it and the [ESP-IDF SDK](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/)
from Espressif (the company that produces the ESP32) used to program it. The specific development board
used in this project is the TinyPICO designed and manufactured by Seon Rozenblum AKA, Unexpected Maker.
This board was chosen in part for its small form factor, making it lightweight and easy to fit on a
small breadboard, and it highly capable ESP32-PICO-D4 SoC. The radio receiver's PWM outputs connect to
standard GPIO inputs on the ESP32 and the IMU communicates over SPI. The ESP32 receives 5V power from
the ESC via the F4 Noxe v3 and outputs the SBUS UART signal to the F4 Noxe v3 via a Sparkfun logic level
converter which converts the 3.3V logic from the ESP32 to the 5V logic required by the F4 Noxe v3. The
flight control software in this repository does not require any additional dependenices or libraries
aside from the ESP-IDF SDK and the standard C library.

<!-- TODO complete wiring diagram -->

_PretoFlyteFC wiring diagram_

The original plan was to assemble all the hardware on a breadboard and then solder it to perf board after
testing was complete, but due to a lack of time, this was not possible and the components remained on the
breadboard in the final design.

![PretoFlyteFC breadboard](assets/breadboard.drawio.png)

_PretoFlyteFC breadboard_

### Control Architecture

![Control Loop](assets/control_architecture.drawio.png)_Control loop_

#### Accelerometer Trigonometry

#### Low pass filter

#### Kalman filter

With a little bit of simplification, a one dimensional Kalman filter looks like the following.

#### PID loop

### Motors and ESC

The quadcopter kit used to test PretoFlyteFC is equipped with brushless DC (BLDC) motors, as most quadcopters are.
Unlike brush DC motors, which are mechanically commutated as the motor spins, BLDC motors must be electronically
commutated, hence the need for the ESC. Firmware on the ESC uses MOSFETs to quickly switch power between the three
phases of the BLDC motors based on the measured back EMF in the unpowered phase. The particular 4-in-1 45A ESC
used in this build also contains a battery elimination circuit (BEC) which means the 7.4V 2s LiPo battery can
connect directly to the ESC and the ESC will in turn supply power to both the motors and rest of electronics
including the flight controller. The ESC abstracts away motor control, and the details how BLDC motors and the
ESC work are largely outside the scope of this project.

The motors are arranged such that adjacent motors rotate in opposite directions and all motors spin inward relative
to the forward orientation of the quadcopter. This balances the torques from each motor so that the quadcopter does
not excessively yaw while hovering. By default, the motors did not spin in the correct directions, so the Betaflight
configuration tool was again used to adjust the direction of rotation.

![Betaflight configuration tool - set motor direction](assets/betaflight_motor_direction.png)_Betaflight configuration tool - set motor direction_

To control a quadcopter, the speed, and thereby thrust, of each motor is varied in the following ways.

- Rise/descend: increase/decrease motor speed equally across all motors
- Pitch: decrease the speed of the front motors, increase the speed of the read motors to pitch forward, vice versa
  for the reverse
- Roll: decrease the speed of the motors on the side to roll towards, increase the speed of the motors on the
  opposite side
- Yaw: increase the speed of the motors on one diagonal, decrease the speed of the motors on the other diagonal,
  motor torques are no longer balanced, creating a net torque on the craft that rotates it
  This sort of control can be achieved with the following motor mixing algorithm. Since the control loop in
  PretoFlyteFC outputs roll, pitch, yaw, and throttle commands, motor mixing takes place on the F4 Noxe v3.

![Motor mixing algorithm](assets/motor_mixing.png)

_Motor mixing algorithm; Source: https://reefwing.medium.com/how-to-write-your-own-flight-controller-software-part-4-8d4c9ce4319_

### Quadcopter Frame

The quadcopter frame is comprised of three carbon fiber plates, four carbon arms, and several metal standoffs
held to together by M3 screws. The carbon fiber plates include mounting holes within the frame for the ESC
and flight controller included in the kit, and there are mounting points for the motors at the end of the arms.
The props are secured to the motor shafts with a single nut that threads onto the shaft such that it tightens
against the prop as the motor spins. The battery was zip-tied to the bottom of the drone to improve the
balance and stability of the system while PretoFlyteFC hardware was zip-tied to the top of the drone.

### Radio Control System

<!-- TODO Interrupt configuration and timing diagram to illustrate -->

To read the PWM signals from the radio receiver, the GPIO inputs were configured with interrupts on both
rising and falling edges. When the an interrupt is triggered on one of these GPIO pins, the current time
in microseconds and GPIO pin level (logic low or high) is recorded by the interrupt handler. The time
between interrupts on the rising and falling edge of the PWM signal is then used to calculate the current
duty cycle given a PWM frequency of 50Hz. The maximum and minimum duty cycle for each channel was measured
using an oscilliscope and thus the duty cycle can be converted to a percentage of range between the minimum
and maximum. These percentages are used as the commanded pitch and roll values.

The ADC was also considered as an alternative to interrupts to read the current PWM values, but this method
was not pursued because of the need to continuously sample and average the PWM signals as well as possible
quantization errors.

### IMU

The SPI driver for the LSM9DS1 was written to more or less use the IMU's default settings. The IMU has one
set of registers for operating the accelerometer and gyroscope and another set of register for the
magnetometer. Because the magnetometer was not used in this project, the driver mainly focuses on the
accelerometer and gyroscope.

The following diagram illustrates how to communicate with the accelerometer and gyroscope over SPI.
Registers can be read from or written to by setting the RW bit and then specifying the 7-bit address of the
register in question. The following byte is the data read from or written to the register.

![LSM9DS1 accelerometer/gyroscope SPI protocol](assets/imu_spi_protocol.png)

_LSM9DS1 accelerometer/gyroscope SPI protocol; Source: https://cdn.sparkfun.com/assets/learn_tutorials/3/7/3/LSM9DS1_Datasheet.pdf_

The driver initializes the accelerometer and gyropscope by writing configuration values to specific control
registers and calibrates the sensors by filling the hardware FIFO on the IMU and averaging the readings to
calculate a bias that is subtracted from all subsequent readings. The driver provides methods to check the
registers that indicate if accelerometer or gyroscope data is available and read any available date from the
X, Y, and Z registers for each sensor.

### SBUS

For a detailed description of the SBUS, see the documentation for the Bolder Flight Systems SBUS
Arduino library found at [https://github.com/bolderflight/sbus](https://github.com/bolderflight/sbus).

Writing a driver to implement the SBUS protocol was fairly straightword since SBUS is a relatively simple
serial communication protocol built on top of UART. SBUS was used to send the roll, pitch, yaw, and throttle
commands output from the PretoFlyteFC control loop to the F4 Noxe v3, which acted as an SBUS-to-DSHOT gateway
between the ESP32 and ESC. The driver uses the SDK to initialize an inverted UART interface. It provides a
method for setting individual channel values and packing and transmitting the bits for each channel according
to the specification found in the detailed SBUS description listed above. Since data only needed to be
sent and not receiver, a single TX/RX UART line from the ESP32 to the F4 Noxe v3 was all that was required to
connect the two controllers. An emergency cutoff switch was also soldered into the green TX/RX line in the
image below away from the props so that could communication could manually terminated as a last resort should
the quadcopter stop responding to inputs from the radio receiver, lose contact with the transmitter, or go
wildly unstable.

![SBUS wiring](assets/sbus_wires.drawio.png)

_SBUS wiring to F4 Noxe v3_

<!-- ### Github Actions -->
<!-- TODO -->

<!-- ### Bill of Materials -->
<!-- TODO -->

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

<!-- Testing and debugging:

- Had to re-pin the connector for the FC/ESC
- Use of logic analyzer
- Plotter tool
- Balance by hand
- Fly by hand for sense of gains
- Betaflight configuraton software to verify signals

- Verified that flight controller received proper SBUS commands using flight controller configuration software (SBUS driver and level shifting)
- Already had two IMU in possession, researched the difficulty for building a driver for each, ended using the older, less sophisticated one because easier to implement driver and simpler to work with
- Verified IMU driver worked with breadboard, printed out pitch and roll
- Verified the drone could be armed and props could be spun using SBUS driver -->

### Re-pinning the flight controller to ESC connector

### IMU driver debugging

### Plotter

### SBUS driver testing with Betaflight configuration tool

### Arming the Drone

### Flying and PID Tuning

<!-- TODO Videos of test flights -->

## Summary, Conclusions, and Future Work

<!-- Write a brief summary of your project and your conclusions from this assignment. Include a detailed
discussion of changes you would make to improve the design if you were to do another design iteration. -->

<!-- Videos of flights -->

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
|   ├── Kalman
|   |   └── include
|   ├── Lsm9ds1
|   |   └── include
│   └── Sbus
|       └── include
├── main
│   └── include
└── tools
    └── cppcheck
```

## Build Instructions

1. Install ESP-IDF SDK
2. Clone repo with submodules
3. Build Cppcheck
4. idf.py build, flash, monitor
5. Make modifications, perform static analysis with Cppcheck
