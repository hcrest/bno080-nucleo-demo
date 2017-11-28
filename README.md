# Example Application for Hillcrest BNO080 Development Kit

This repository contains the example application for the Hillcrest
BNO080 development kit.

The code runs on an ST Nucleo evaluation board combined with the BNO
shield board by Hillcrest.  This shield incorporates the BNO080
sensor hub, enabling SH-2 functionality for the demo system.

## Requirements

* IAR Embedded Workbench for ARM (EWARM) version 7.4
* STM32F411 Nucleo board
* Hillcrest BNO080 Shield board

## Setup

Clone this repository using the --recursive flag with git:
  * git clone --recursive https://github.com/hcrest/bno080-nucleo-demo.git

## Building the Code
* Use IAR EWARM to open the workspace, bno080-nucleo-demo/EWARM/Project.eww
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
reading and printing Rotation Vectors from the sensor hub:

```

Hillcrest SH-2 Demo.
SH2 Reset.
Part 10003608 : Version 3.2.7 Build 370
Part 10003606 : Version 1.2.4 Build 230
Starting Sensor Reports.
  0.3000 Rotation Vector: r:0.025 i:0.773 j:0.633 k:-0.039 (acc: 180.001 deg)
  0.3099 Rotation Vector: r:0.025 i:0.773 j:0.633 k:-0.039 (acc: 180.001 deg)
  0.3210 Rotation Vector: r:0.025 i:0.773 j:0.633 k:-0.039 (acc: 180.001 deg)
  0.3305 Rotation Vector: r:0.025 i:0.773 j:0.633 k:-0.039 (acc: 180.001 deg)
  0.3409 Rotation Vector: r:0.025 i:0.773 j:0.633 k:-0.039 (acc: 180.001 deg)
.
.
.
```

## Saleae Logic Analyzer Captures

In the LogicCaptures folder, there are example logic analyzer traces of the
BNO080's operation with I2C, SPI and UART connections.  See the README.txt
file located there, for instructions on downloading the viewer application.
