# Keyboard Activity Sensor
A LIS3DH IMU sensor connected to a teensyduino running and attached to a keyboard to measure vibrations from key presses. The teensyduino should output key presses to a controlling computer. This code is derived from the tap demo and tutorial information on [Adafruit](https://learn.adafruit.com/adafruit-lis3dh-triple-axis-accelerometer-breakout/arduino)

## Hardware
- [Teensy 3.2](https://www.pjrc.com/store/teensy32.html)
- [LIS3DH accelerometer](https://www.adafruit.com/product/2809)
- [Push Button Casing](https://www.thingiverse.com/thing:3195198)
- 12mm square x 10mm tall tactile switch (standardized part)
- Circuit Housing (3D print files)

## Software Requirements
- [Arduino IDE](https://www.arduino.cc/)
- [Teensyduino](https://www.pjrc.com/teensy/td_download.html) (Used Teensy microcontroller version 3.2)
- Adafruit LIS3DH library from the library manager
- Adafruit Unified Sensor library from the library manager
- Adafruit BusIO library from the library manager
- Arduino basic Statistics package from the library manager by [Rob Tillaart](https://github.com/RobTillaart/Arduino/tree/master/libraries/Statistic)


## Notes
- Set debugging to true to see print statements

## References and Source
- [Button Class Code](https://roboticsbackend.com/arduino-object-oriented-programming-oop/)
