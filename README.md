# Zumo1

Firmware for Zumo and Teensy 3.1 based mini robot.
[Main GIT repository](https://github.com/10bulls/zumo1)

This project makes use of a customised build of **micropython** for Teensy.
This was added when micropython for Teensy was in early development as at 2014-02-19.
Details for adding this library can be found [here](#micropython-for-teensy).



## Hardware part list

- Zumo shield
- [Teensy 3.2 microcontroller](#teensy-3.2)
- IMU Pololu imu01b 
- Micro-SD adafruit breakout 
- bluetooth radio
- Sharp distance sensor
- IR remote receiver

## Teensy 3.2

[Teensy 3.2 microcontroller website](https://www.pjrc.com/store/teensy32.html)

### Pinout (front)
![Pinout (front)](https://www.pjrc.com/teensy/teensy32_front_pinout.png)

### Pinout (rear)
![Pinout (rear)](https://www.pjrc.com/teensy/teensy32_back_pinout.png)

## Micropython for teensy

[Main GIT repository for libmpython](https://github.com/10bulls/libmpython)

A working build of libmpython (the micropython for Teensy library) is included in the 
/library/mpython folder of the Zumo1 repository.

Copy the _mpython_ folder to the arduino libaries folder.

Add the contents of 'libraries/boards.txt' to the existing 'arduino/hardware/teensy/boards.txt'  
**DO NOT REPLACE THE EXISTING ONE!**  
This file contains hard coded paths to the location of **libmpython.a** that will need to be modified.

Copy 'libraries/mk20dx256py.ld' to 'arduino/hardware/teensy/cores/teensy3'

Needs Teensyduino 1.18 RC # 2 or later for programs > 128K
http://forum.pjrc.com/threads/24796-Teensyduino-1-18-Release-Candidate-2-Available

Python support can then be added to a project by simply selecting **Teensy 3.1+Python** as the target board.
