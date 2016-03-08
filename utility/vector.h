/*
    Inertial Measurement Unit Maths Library
    Copyright (C) 2013-2014  Samuel Cowen
    www.camelsoftware.com

    Bug fixes and cleanups by Gé Vissers (gvissers@gmail.com)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef IMUMATH_VECTOR_HPP
#define IMUMATH_VECTOR_HPP

#include <string.h>
#include <stdint.h>
#include <math.h>


namespace imu
{

template <uint8_t N> class Vector
{
public:
    Vector()
    {
        memset(p_vec, 0, sizeof(double)*N);
    }

    Vector(double a)
    {
        memset(p_vec, 0, sizeof(double)*N);
        p_vec[0] = a;
    }

    Vector(double a, double b)
    {
        memset(p_vec, 0, sizeof(double)*N);
        p_vec[0] = a;
        p_vec[1] = b;
    }

    Vector(double a, double b, double c)
    {
        memset(p_vec, 0, sizeof(double)*N);
        p_vec[0] = a;
        p_vec[1] = b;
        p_vec[2] = c;
    }

    Vector(double a, double b, double c, double d)
    {
        memset(p_vec, 0, sizeof(double)*N);
        p_vec[0] = a;
        p_vec[1] = b;
        p_vec[2] = c;
        p_vec[3] = d;
    }

    Vector(const Vector<N> &v)
    {
        for (int x = 0; x < N; x++)
            p_vec[x] = v.p_vec[x];
    }

    ~Vector()
    {
    }

    uint8_t n() { return N; }

    double magnitude() const
    {
        double res = 0;
        for (int i = 0; i < N; i++)
            res += p_vec[i] * p_vec[i];

        return sqrt(res);
    }

    void normalize()
    {
        double mag = magnitude();
        if (isnan(mag) || mag == 0.0)
            return;

        for (int i = 0; i < N; i++)
            p_vec[i] /= mag;
    }

    double dot(const Vector& v) const
    {
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
    Vector cross(const Vector& v) const;

    Vector scale(double scalar) const
    {
        Vector ret;
        for(int i = 0; i < N; i++)
            ret.p_vec[i] = p_vec[i] * scalar;
        return ret;
    }

    Vector invert() const
    {
        Vector ret;
        for(int i = 0; i < N; i++)
            ret.p_vec[i] = -p_vec[i];
        return ret;
    }

    Vector& operator=(const Vector& v)
    {
        for (int x = 0; x < N; x++ )
            p_vec[x] = v.p_vec[x];
        return *this;
    }

    double& operator [](int n)
    {
        return p_vec[n];
    }

    double operator [](int n) const
    {
        return p_vec[n];
    }

    double& operator ()(int n)
    {
        return p_vec[n];
    }

    double operator ()(int n) const
    {
        return p_vec[n];
    }

    Vector operator+(const Vector& v) const
    {
        Vector ret;
        for(int i = 0; i < N; i++)
            ret.p_vec[i] = p_vec[i] + v.p_vec[i];
        return ret;
    }

    Vector operator-(const Vector& v) const
    {
        Vector ret;
        for(int i = 0; i < N; i++)
            ret.p_vec[i] = p_vec[i] - v.p_vec[i];
        return ret;
    }

    Vector operator * (double scalar) const
    {
        return scale(scalar);
    }

    Vector operator / (double scalar) const
    {
        Vector ret;
        for(int i = 0; i < N; i++)
            ret.p_vec[i] = p_vec[i] / scalar;
        return ret;
    }

    void toDegrees()
    {
        for(int i = 0; i < N; i++)
            p_vec[i] *= 57.2957795131; //180/pi
    }

    void toRadians()
    {
        for(int i = 0; i < N; i++)
            p_vec[i] *= 0.01745329251;  //pi/180
    }

    double& x() { return p_vec[0]; }
    double& y() { return p_vec[1]; }
    double& z() { return p_vec[2]; }
    double x() const { return p_vec[0]; }
    double y() const { return p_vec[1]; }
    double z() const { return p_vec[2]; }


private:
    double p_vec[N];
};


template <>
inline Vector<3> Vector<3>::cross(const Vector& v) const
{
    return Vector(
        p_vec[1] * v.p_vec[2] - p_vec[2] * v.p_vec[1],
        p_vec[2] * v.p_vec[0] - p_vec[0] * v.p_vec[2],
        p_vec[0] * v.p_vec[1] - p_vec[1] * v.p_vec[0]
    );
}

} // namespace

#endif
