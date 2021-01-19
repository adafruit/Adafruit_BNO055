/*!
 *  @file imu.h
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

/*!
 *  @brief  Class that stores orientation information in it
 */
class Quaternion {
public:
  /*!
   *  @brief  Instantiates a new Quaternion class to the identity
   */
  Quaternion() : w(1.0), x(0.0), y(0.0), z(0.0) {}

  /*!
   *  @brief  Instantiates a new Quaternion class
   *  @param  ww
   *          real component
   *  @param  xx
   *          imaginary x
   *  @param  yy
   *          imaginary y
   *  @param  zz
   *          imaginary z
   */
  Quaternion(double ww, double xx, double yy, double zz)
      : w(ww), x(xx), y(yy), z(zz) {}

  /*!
   *  @brief  Calculates magnitude
   *  @return magnitude of quaternion
   */
  double magnitude() const { return sqrt(w * w + x * x + y * y + z * z); }
  /*!
   *  @brief  Normalizes the quaternion
   */
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

  double w; /*!< real */
  double x; /*!< x */
  double y; /*!< y */
  double z; /*!< z */
};

/*!
 *  @brief  Class that stores vector information in it
 */
class Vector {
public:
  /*!
   *  @brief  Instantiates a new Vector class to (0,0,0)
   */
  Vector() : x(0.0), y(0.0), z(0.0) {}

  /*!
   *  @brief  Instantiates a new Vector class to (0,0,0)
   *  @param  xx
   *          x
   *  @param  yy
   *          y
   *  @param  zz
   *          z
   */
  Vector(double xx, double yy, double zz) : x(xx), y(yy), z(zz) {}

  double x; /*!< x */
  double y; /*!< y */
  double z; /*!< z */
};

}; // namespace imu
