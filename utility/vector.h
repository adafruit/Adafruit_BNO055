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

#ifndef IMUMATH_VECTOR_HPP
#define IMUMATH_VECTOR_HPP

#include <math.h>
#include <stdint.h>
#include <string.h>

namespace imu {

template <uint8_t N> class Vector {
public:
  Vector() { memset(p_vec, 0, sizeof(double) * N); }

  Vector(double a) {
    memset(p_vec, 0, sizeof(double) * N);
    p_vec[0] = a;
  }

  Vector(double a, double b) {
    memset(p_vec, 0, sizeof(double) * N);
    p_vec[0] = a;
    p_vec[1] = b;
  }

  Vector(double a, double b, double c) {
    memset(p_vec, 0, sizeof(double) * N);
    p_vec[0] = a;
    p_vec[1] = b;
    p_vec[2] = c;
  }

  Vector(double a, double b, double c, double d) {
    memset(p_vec, 0, sizeof(double) * N);
    p_vec[0] = a;
    p_vec[1] = b;
    p_vec[2] = c;
    p_vec[3] = d;
  }

  Vector(const Vector<N> &v) {
    for (int x = 0; x < N; x++)
      p_vec[x] = v.p_vec[x];
  }

  ~Vector() {}

  uint8_t n() { return N; }

  double magnitude() const {
    double res = 0;
    for (int i = 0; i < N; i++)
      res += p_vec[i] * p_vec[i];

    return sqrt(res);
  }

  void normalize() {
    double mag = magnitude();
    if (isnan(mag) || mag == 0.0)
      return;

    for (int i = 0; i < N; i++)
      p_vec[i] /= mag;
  }

  double dot(const Vector &v) const {
    double ret = 0;
    for (int i = 0; i < N; i++)
      ret += p_vec[i] * v.p_vec[i];

    return ret;
  }

  // The cross product is only valid for vectors with 3 dimensions,
  // with the exception of higher dimensional stuff that is beyond
  // the intended scope of this library.
  // Only a definition for N==3 is given below this class, using
  // cross() with another value for N will result in a link error.
  Vector cross(const Vector &v) const;

  Vector scale(double scalar) const {
    Vector ret;
    for (int i = 0; i < N; i++)
      ret.p_vec[i] = p_vec[i] * scalar;
    return ret;
  }

  Vector invert() const {
    Vector ret;
    for (int i = 0; i < N; i++)
      ret.p_vec[i] = -p_vec[i];
    return ret;
  }

  Vector &operator=(const Vector &v) {
    for (int x = 0; x < N; x++)
      p_vec[x] = v.p_vec[x];
    return *this;
  }

  double &operator[](int n) { return p_vec[n]; }

  double operator[](int n) const { return p_vec[n]; }

  double &operator()(int n) { return p_vec[n]; }

  double operator()(int n) const { return p_vec[n]; }

  Vector operator+(const Vector &v) const {
    Vector ret;
    for (int i = 0; i < N; i++)
      ret.p_vec[i] = p_vec[i] + v.p_vec[i];
    return ret;
  }

  Vector operator-(const Vector &v) const {
    Vector ret;
    for (int i = 0; i < N; i++)
      ret.p_vec[i] = p_vec[i] - v.p_vec[i];
    return ret;
  }

  Vector operator*(double scalar) const { return scale(scalar); }

  Vector operator/(double scalar) const {
    Vector ret;
    for (int i = 0; i < N; i++)
      ret.p_vec[i] = p_vec[i] / scalar;
    return ret;
  }

  void toDegrees() {
    for (int i = 0; i < N; i++)
      p_vec[i] *= 57.2957795131; // 180/pi
  }

  void toRadians() {
    for (int i = 0; i < N; i++)
      p_vec[i] *= 0.01745329251; // pi/180
  }

  double &x() { return p_vec[0]; }
  double &y() { return p_vec[1]; }
  double &z() { return p_vec[2]; }
  double x() const { return p_vec[0]; }
  double y() const { return p_vec[1]; }
  double z() const { return p_vec[2]; }

private:
  double p_vec[N];
};

template <> inline Vector<3> Vector<3>::cross(const Vector &v) const {
  return Vector(p_vec[1] * v.p_vec[2] - p_vec[2] * v.p_vec[1],
                p_vec[2] * v.p_vec[0] - p_vec[0] * v.p_vec[2],
                p_vec[0] * v.p_vec[1] - p_vec[1] * v.p_vec[0]);
}

} // namespace imu

#endif
