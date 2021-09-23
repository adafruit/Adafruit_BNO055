//  Inertial Measurement Unit Maths Library
//
//  Copyright 2013-2021 Sam Cowen <samuel.cowen@camelsoftware.com>
//  Bug fixes and cleanups by Gé Vissers (gvissers@gmail.com)
//
//  Permission is hereby granted, free of charge, to any person obtaining a
//  copy of this software and associated documentation files (the "Software"),
//  to deal in the Software without restriction, including without limitation
//  the rights to use, copy, modify, merge, publish, distribute, sublicense,
//  and/or sell copies of the Software, and to permit persons to whom the
//  Software is furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
//  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//  DEALINGS IN THE SOFTWARE.

#ifndef IMUMATH_QUATERNION_HPP
#define IMUMATH_QUATERNION_HPP

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "matrix.h"

namespace imu {

class Quaternion {
public:
  Quaternion() : _w(1.0), _x(0.0), _y(0.0), _z(0.0) {}

  Quaternion(double w, double x, double y, double z)
      : _w(w), _x(x), _y(y), _z(z) {}

  Quaternion(double w, Vector<3> vec)
      : _w(w), _x(vec.x()), _y(vec.y()), _z(vec.z()) {}

  double &w() { return _w; }
  double &x() { return _x; }
  double &y() { return _y; }
  double &z() { return _z; }

  double w() const { return _w; }
  double x() const { return _x; }
  double y() const { return _y; }
  double z() const { return _z; }

  double magnitude() const {
    return sqrt(_w * _w + _x * _x + _y * _y + _z * _z);
  }

  void normalize() {
    double mag = magnitude();
    *this = this->scale(1 / mag);
  }

  Quaternion conjugate() const { return Quaternion(_w, -_x, -_y, -_z); }

  void fromAxisAngle(const Vector<3> &axis, double theta) {
    _w = cos(theta / 2);
    // only need to calculate sine of half theta once
    double sht = sin(theta / 2);
    _x = axis.x() * sht;
    _y = axis.y() * sht;
    _z = axis.z() * sht;
  }

  void fromMatrix(const Matrix<3> &m) {
    double tr = m.trace();

    double S;
    if (tr > 0) {
      S = sqrt(tr + 1.0) * 2;
      _w = 0.25 * S;
      _x = (m(2, 1) - m(1, 2)) / S;
      _y = (m(0, 2) - m(2, 0)) / S;
      _z = (m(1, 0) - m(0, 1)) / S;
    } else if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2)) {
      S = sqrt(1.0 + m(0, 0) - m(1, 1) - m(2, 2)) * 2;
      _w = (m(2, 1) - m(1, 2)) / S;
      _x = 0.25 * S;
      _y = (m(0, 1) + m(1, 0)) / S;
      _z = (m(0, 2) + m(2, 0)) / S;
    } else if (m(1, 1) > m(2, 2)) {
      S = sqrt(1.0 + m(1, 1) - m(0, 0) - m(2, 2)) * 2;
      _w = (m(0, 2) - m(2, 0)) / S;
      _x = (m(0, 1) + m(1, 0)) / S;
      _y = 0.25 * S;
      _z = (m(1, 2) + m(2, 1)) / S;
    } else {
      S = sqrt(1.0 + m(2, 2) - m(0, 0) - m(1, 1)) * 2;
      _w = (m(1, 0) - m(0, 1)) / S;
      _x = (m(0, 2) + m(2, 0)) / S;
      _y = (m(1, 2) + m(2, 1)) / S;
      _z = 0.25 * S;
    }
  }

  void toAxisAngle(Vector<3> &axis, double &angle) const {
    double sqw = sqrt(1 - _w * _w);
    if (sqw == 0) // it's a singularity and divide by zero, avoid
      return;

    angle = 2 * acos(_w);
    axis.x() = _x / sqw;
    axis.y() = _y / sqw;
    axis.z() = _z / sqw;
  }

  Matrix<3> toMatrix() const {
    Matrix<3> ret;
    ret.cell(0, 0) = 1 - 2 * _y * _y - 2 * _z * _z;
    ret.cell(0, 1) = 2 * _x * _y - 2 * _w * _z;
    ret.cell(0, 2) = 2 * _x * _z + 2 * _w * _y;

    ret.cell(1, 0) = 2 * _x * _y + 2 * _w * _z;
    ret.cell(1, 1) = 1 - 2 * _x * _x - 2 * _z * _z;
    ret.cell(1, 2) = 2 * _y * _z - 2 * _w * _x;

    ret.cell(2, 0) = 2 * _x * _z - 2 * _w * _y;
    ret.cell(2, 1) = 2 * _y * _z + 2 * _w * _x;
    ret.cell(2, 2) = 1 - 2 * _x * _x - 2 * _y * _y;
    return ret;
  }

  // Returns euler angles that represent the quaternion.  Angles are
  // returned in rotation order and right-handed about the specified
  // axes:
  //
  //   v[0] is applied 1st about z (ie, roll)
  //   v[1] is applied 2nd about y (ie, pitch)
  //   v[2] is applied 3rd about x (ie, yaw)
  //
  // Note that this means result.x() is not a rotation about x;
  // similarly for result.z().
  //
  Vector<3> toEuler() const {
    Vector<3> ret;
    double sqw = _w * _w;
    double sqx = _x * _x;
    double sqy = _y * _y;
    double sqz = _z * _z;

    ret.x() = atan2(2.0 * (_x * _y + _z * _w), (sqx - sqy - sqz + sqw));
    ret.y() = asin(-2.0 * (_x * _z - _y * _w) / (sqx + sqy + sqz + sqw));
    ret.z() = atan2(2.0 * (_y * _z + _x * _w), (-sqx - sqy + sqz + sqw));

    return ret;
  }

  Vector<3> toAngularVelocity(double dt) const {
    Vector<3> ret;
    Quaternion one(1.0, 0.0, 0.0, 0.0);
    Quaternion delta = one - *this;
    Quaternion r = (delta / dt);
    r = r * 2;
    r = r * one;

    ret.x() = r.x();
    ret.y() = r.y();
    ret.z() = r.z();
    return ret;
  }

  Vector<3> rotateVector(const Vector<2> &v) const {
    return rotateVector(Vector<3>(v.x(), v.y()));
  }

  Vector<3> rotateVector(const Vector<3> &v) const {
    Vector<3> qv(_x, _y, _z);
    Vector<3> t = qv.cross(v) * 2.0;
    return v + t * _w + qv.cross(t);
  }

  Quaternion operator*(const Quaternion &q) const {
    return Quaternion(_w * q._w - _x * q._x - _y * q._y - _z * q._z,
                      _w * q._x + _x * q._w + _y * q._z - _z * q._y,
                      _w * q._y - _x * q._z + _y * q._w + _z * q._x,
                      _w * q._z + _x * q._y - _y * q._x + _z * q._w);
  }

  Quaternion operator+(const Quaternion &q) const {
    return Quaternion(_w + q._w, _x + q._x, _y + q._y, _z + q._z);
  }

  Quaternion operator-(const Quaternion &q) const {
    return Quaternion(_w - q._w, _x - q._x, _y - q._y, _z - q._z);
  }

  Quaternion operator/(double scalar) const {
    return Quaternion(_w / scalar, _x / scalar, _y / scalar, _z / scalar);
  }

  Quaternion operator*(double scalar) const { return scale(scalar); }

  Quaternion scale(double scalar) const {
    return Quaternion(_w * scalar, _x * scalar, _y * scalar, _z * scalar);
  }

private:
  double _w, _x, _y, _z;
};

} // namespace imu

#endif
