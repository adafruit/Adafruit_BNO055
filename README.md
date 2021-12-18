Adafruit Unified BNO055 Driver (AHRS/Orientation)  [![Build Status](https://github.com/adafruit/Adafruit_BNO055/workflows/Arduino%20Library%20CI/badge.svg)](https://github.com/adafruit/Adafruit_BNO055/actions)[![Documentation](https://github.com/adafruit/ci-arduino/blob/master/assets/doxygen_badge.svg)](http://adafruit.github.io/Adafruit_BNO055/html/index.html)
================

<a href="https://www.adafruit.com/product/2472"><img src="assets/board.jpg?raw=true" width="500px"></a>

This driver is for the Adafruit BNO055 Breakout, and is based on Adafruit's Unified Sensor Library (Adafruit_Sensor).

Tested and works great with the Adafruit Si4713 Breakout Board.
To work with the Arduino Zero, the BNO055's ADR pin must be high.
* http://www.adafruit.com/products/2472

## What is the Adafruit Unified Sensor Library? ##

The Adafruit Unified Sensor Library ([Adafruit_Sensor](https://github.com/adafruit/Adafruit_Sensor)) provides a common interface and data type for any supported sensor.  It defines some basic information about the sensor (sensor limits, etc.), and returns standard SI units of a specific type and scale for each supported sensor type.

It provides a simple abstraction layer between your application and the actual sensor HW, allowing you to drop in any comparable sensor with only one or two lines of code to change in your project (essentially the constructor since the functions to read sensor data and get information about the sensor are defined in the base Adafruit_Sensor class).

This is important and useful for two reasons:

1.) You can use the data right away because it's already converted to SI units that you understand and can compare, rather than meaningless values like 0..1023.

2.) Because SI units are standardised in the sensor library, you can also do quick sanity checks when working with new sensors, or drop in any comparable sensor if you need better sensitivity or if a lower cost unit becomes available, etc. 

Light sensors will always report units in lux, gyroscopes will always report units in rad/s, etc. ... freeing you up to focus on the data, rather than digging through the datasheet to understand what the sensor's raw numbers really mean.

Adafruit invests time and resources providing this open source code.  Please support Adafruit and open-source hardware by purchasing products from Adafruit!

Kevin (KTOWN) Townsend Adafruit Industries.
MIT license, check license.txt for more information
All text above must be included in any redistribution

To install, use the Arduino Library Manager and search for "Adafruit BNO055" and install the library.
