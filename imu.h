/*!
 *  @file Adafruit_BNO055.h
 *
 *  This is a library for the BNO055 orientation sensor
 *
 *  Designed specifically to work with the Adafruit BNO055 Breakout.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/2472
 *
 *  These sensors use I2C to communicate, 2 pins are required to interface.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  Kevin Walchko (for Adafruit Industries)
 *
 *  MIT license, all text above must be included in any redistribution
 */

#pragma once

#include <math.h>

namespace imu {

class Quaternion {
public:
  Quaternion() : w(1.0), x(0.0), y(0.0), z(0.0) {}

  Quaternion(double ww, double xx, double yy, double zz)
      : w(ww), x(xx), y(yy), z(zz) {}

  double magnitude() const { return sqrt(w * w + x * x + y * y + z * z); }

  void normalize() {
    double mag = magnitude();
    if (mag < 1e-10)
      return;
    double inv = 1.0 / mag;
    w *= inv;
    x *= inv;
    y *= inv;
    z *= inv;
  }

  double w, x, y, z;
};

class Vector {
public:
  Vector() : x(0.0), y(0.0), z(0.0) {}
  Vector(double xx, double yy, double zz) : x(xx), y(yy), z(zz) {}

  double x, y, z;
};

}; // namespace imu
