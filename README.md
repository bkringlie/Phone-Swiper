# Phone-Swiper

## Introduction
Phone Swiper is an innovative project that utilizes two lidar sensors, two servo motors, a stylus pen, and an Arduino Nano to interact with touch screen devices. This project can be particularly useful in automated testing of touch screen applications or for accessibility purposes.

## Setup and Installation
### Hardware Requirements
- 2 x Lidar sensors
- 2 x Servo motors
- 1 x Stylus pen
- 1 x Arduino Nano
- Necessary cables and mounting hardware

### Software Requirements
- Arduino IDE
- VL53L1X library for Arduino
- Servo library for Arduino

### Assembly Instructions
1. Attach the servo motors in a double-jointed setup.
2. Connect the lidar sensors to the Arduino Nano according to the schematic provided in the `hardware` folder.
3. Mount the stylus pen to one of the servo arms.

## Usage
1. Upload the `Phone Swiper - Spring 2022.ino` sketch to your Arduino Nano using the Arduino IDE.
2. Position the device in front of a touch screen.
3. The device will swipe right or left on the screen when an object is detected within 100mm of either lidar sensor.

## Credits
- Authors: Theodore Wilson, Mason Greseth, Brandt Kringlie
- Lidar Library: Pololu
- Template Code: David Orser
