# PretoFlyteFC

Simple quadcopter flight controller based on an ESP32 and IMU

## Table of Contents

TODO re-organize table of contents (repo org and build instr after design overview?)

- [Design Overview](#design-overview)
- [Preliminary Design Verification](#preliminary-design-verification)
- [Design Implementation](#design-implementation)
- [Design Testing](#design-testing)
- [Summary, Conclusions, and Future Work](#summary-conclusions-and-future-work)
- [Repository Organization](#repository-organization)
- [Build Instructions](#build-instructions)

## Design Overview

In this section, you should provide a detailed description of your design.

- Start with a high-level description of your design and its purpose.
- Discuss the original design concepts that you considered (at a high level) and then your
  final design. This document is not only a description of the technical aspects of your
  design, but it is also a digest of your design process.
- Clearly explain how your project expands and builds on previous works (i.e. justifications
  for a 6-week project)
- Don’t forget to cite your references and include images/schematics.

## Preliminary Design Verification

Write a summary of preliminary testing results from your prototypes in this section.

What did you to do verify your design before moving into the final production phase (e.g. breadboards, test programs, etc)?

What steps did you take, if any, to determine the feasibility of your design?

Explain your test plan, the procedures you use to test your design, the various subcomponents and the outcomes from those tests. Include photos of the initial, assembled prototype, and photos from any testing results if available. This includes verification of hardware, software and enclosures.

- Assembled frame, soldered ESC and secondary flight controller, verified motors could manually be spun from flight controller configuration software
- Tested if ESC could directly be commanded with ESP32 (Arduino PWM program)
- Verified that flight controller received proper SBUS commands using flight controller configuration software (SBUS driver and level shifting)
- Already had two IMU in possession, researched the difficulty for building a driver for each, ended using the older, less sophisticated one because easier to implement driver and simpler to work with
- Verified IMU driver worked with breadboard, printed out pitch and roll
- Verified the drone could be armed and props could be spun using SBUS driver

## Design Implementation

In this section, you will explain the final design you arrived at. Include

- Include an overview of the overall system, described at a high level
- Include a listing of all the relevant subcomponents developed/used in creating the
  final software/hardware
- If there is anything else that is notable about your design, or design practices, please
  include it in this section. Discuss your design process in developing the system and
  any challenges that came about.

## Design Testing

Write a summary of testing results from your final prototype in this section.

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
  be in either the report, or the final presentation.

Testing and debugging:

- Had to re-pin the connector for the FC/ESC
- Use of logic analyzer
- Plotter tool

## Summary, Conclusions, and Future Work

Write a brief summary of your project and your conclusions from this assignment. Include a detailed
discussion of changes you would make to improve the design if you were to do another design iteration.

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
