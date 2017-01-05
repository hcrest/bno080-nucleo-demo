# Example Application for Hillcrest BNO080 Development Kit

This repository contains the example application for the Hillcrest
BNO080 development kit.

The code runs on an ST Nucleo evaluation board combined with the BNO
shield board by Hillcrest.  This shield incorporates the BNO080
SensorHub, enabling SH-2 SensorHub functionality for the demo system.

## Requirements

* IAR Embedded Workbench for ARM (EWARM) version 7.4
* STM32F411 Nucleo board
* Hillcrest BNO080 Shield board

## Setup

Clone this repository using the --recursive flag with git:
  * git clone --recursive http://github.hcrest.com/hillcrest/sh2-example-nucleo

## Building the Code
* Use IAR EWARM to open the workspace, sh2-example-nucleo/EWARM/Project.eww
* Select the sh2-demo-i2c or sh2-demo-spi project configuration.
* Run Project -> Rebuild All to compile the project.

## Running the Application

* Mount the shield board on the Nucleo platform.
* Ensure all switches are set properly (switch positions vary
  based on whether SPI or I2C interface is used.)
* Connect the Nucleo board to the development PC via USB.
* In IAR EWARM, execute Project -> Download and Debug.
* Once the debugger is ready, click the Go button.

The application should print the SH-2 version numbers, then start
reading and printing Rotation Vectors from the SensorHub:

```

Hillcrest SH-2 Demo.
Part 10003608 : Version 3.2.4 Build 283
Part 10003606 : Version 1.2.3 Build 206
Reset Event.
Exec Reset Event, Starting reports.
  0.3756 Rotation Vector: r:0.931 i:-0.013 j:0.014 k:-0.364 (acc: 180.001 deg)
  0.3858 Rotation Vector: r:0.931 i:-0.013 j:0.014 k:-0.364 (acc: 180.001 deg)
  0.3958 Rotation Vector: r:0.931 i:-0.013 j:0.014 k:-0.364 (acc: 180.001 deg)
  0.4058 Rotation Vector: r:0.931 i:-0.013 j:0.014 k:-0.364 (acc: 180.001 deg)
  0.4159 Rotation Vector: r:0.931 i:-0.013 j:0.014 k:-0.364 (acc: 180.001 deg)
.
.
.
```
