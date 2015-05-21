/*
    Inertial Measurement Unit Maths Library
    Copyright (C) 2013-2014  Samuel Cowen
    www.camelsoftware.com

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

#include <stdlib.h>
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

    double magnitude()
    {
        double res = 0;
        int i;
        for(i = 0; i < N; i++)
            res += (p_vec[i] * p_vec[i]);

        if(isnan(res))
            return 0;
        if((fabs(res-1)) >= 0.000001) // Avoid a sqrt if possible.
            return sqrt(res);
        return 1;
    }

    void normalize()
    {
        double mag = magnitude();
        if(abs(mag) <= 0.0001)
            return;

        int i;
        for(i = 0; i < N; i++)
            p_vec[i] = p_vec[i]/mag;
    }

    double dot(Vector v)
    {
        double ret = 0;
        int i;
        for(i = 0; i < N; i++)
            ret += p_vec[i] * v.p_vec[i];

        return ret;
    }

    Vector cross(Vector v)
    {
        Vector ret;

        // The cross product is only valid for vectors with 3 dimensions,
        // with the exception of higher dimensional stuff that is beyond the intended scope of this library
        if(N != 3)
            return ret;

        ret.p_vec[0] = (p_vec[1] * v.p_vec[2]) - (p_vec[2] * v.p_vec[1]);
        ret.p_vec[1] = (p_vec[2] * v.p_vec[0]) - (p_vec[0] * v.p_vec[2]);
        ret.p_vec[2] = (p_vec[0] * v.p_vec[1]) - (p_vec[1] * v.p_vec[0]);
        return ret;
    }

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

    Vector operator = (Vector v)
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

    Vector operator + (Vector v) const
    {
        Vector ret;
        for(int i = 0; i < N; i++)
            ret.p_vec[i] = p_vec[i] + v.p_vec[i];
        return ret;
    }

    Vector operator - (Vector v) const
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
    double  p_vec[N];
};


};

#endif
