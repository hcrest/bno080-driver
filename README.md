# NOTICE: Obsolete Code

This repository is no longer maintained.

The currently supported driver for the BNO080 and related SH2 sensor
hubs can be found at https://github.com/ceva-dsp/sh2.  And an example
project that demonstrates these modules and their driver are at
https://github.com/ceva-dsp/sh2-demo-nucleo.

# SensorHub Driver for MCU Applications

The files in this repository provide application-level SH-2 sensor hub functionality.

To use this code, an application developer will need to:
* Incorporate this code into a project.
* Provide platform-level functions, as specified in sh2_hal.h
* Develop application logic to call the functions in sh2.h

More complete instruction can be found in the User's Guide:
* [BNO080 driver User's Guide](UserGuide.pdf)

An example project based on this driver can be found here:
* [bno080-nucleo-demo](https://github.com/hcrest/bno080-nucleo-demo)


