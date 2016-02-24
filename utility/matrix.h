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

#ifndef IMUMATH_MATRIX_HPP
#define IMUMATH_MATRIX_HPP

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "vector.h"

namespace imu
{


template <uint8_t N> class Matrix
{
public:
    Matrix()
    {
        _cell = &_cell_data[0];
        memset(_cell, 0, N*N*sizeof(double));
    }

    Matrix(const Matrix &m)
    {
        _cell = &_cell_data[0];
        for (int ij = 0; ij < N*N; ++ij)
        {
            _cell_data[ij] = m._cell_data[ij];
        }
    }

    ~Matrix()
    {
    }

    Matrix& operator=(const Matrix& m)
    {
        for (int ij = 0; ij < N*N; ++ij)
        {
            _cell_data[ij] = m._cell_data[ij];
        }
        return *this;
    }

    Vector<N> row_to_vector(int i) const
    {
        Vector<N> ret;
        for (int j = 0; j < N; j++)
        {
            ret[j] = cell(i, j);
        }
        return ret;
    }

    Vector<N> col_to_vector(int j) const
    {
        Vector<N> ret;
        for (int i = 0; i < N; i++)
        {
            ret[i] = cell(i, j);
        }
        return ret;
    }

    void vector_to_row(const Vector<N>& v, int i)
    {
        for (int j = 0; j < N; j++)
        {
            cell(i, j) = v[j];
        }
    }

    void vector_to_col(const Vector<N>& v, int j)
    {
        for (int i = 0; i < N; i++)
        {
            cell(i, j) = v[i];
        }
    }

    double operator()(int i, int j) const
    {
        return cell(i, j);
    }
    double& operator()(int i, int j)
    {
        return cell(i, j);
    }

    double cell(int i, int j) const
    {
        return _cell_data[i*N+j];
    }
    double& cell(int i, int j)
    {
        return _cell_data[i*N+j];
    }


    Matrix operator+(const Matrix& m) const
    {
        Matrix ret;
        for (int ij = 0; ij < N*N; ++ij)
        {
            ret._cell_data[ij] = _cell_data[ij] + m._cell_data[ij];
        }
        return ret;
    }

    Matrix operator-(const Matrix& m) const
    {
        Matrix ret;
        for (int ij = 0; ij < N*N; ++ij)
        {
            ret._cell_data[ij] = _cell_data[ij] - m._cell_data[ij];
        }
        return ret;
    }

    Matrix operator*(double scalar) const
    {
        Matrix ret;
        for (int ij = 0; ij < N*N; ++ij)
        {
            ret._cell_data[ij] = _cell_data[ij] * scalar;
        }
        return ret;
    }

    Matrix operator*(const Matrix& m) const
    {
        Matrix ret;
        for (int i = 0; i < N; i++)
        {
            Vector<N> row = row_to_vector(i);
            for (int j = 0; j < N; j++)
            {
                ret.cell(i, j) = row.dot(m.col_to_vector(j));
            }
        }
        return ret;
    }

    Matrix transpose()
    {
        Matrix ret;
        for(int x = 0; x < N; x++)
        {
            for(int y = 0; y < N; y++)
            {
                ret.cell(y, x) = cell(x, y);
            }
        }
        return ret;
    }

    Matrix<N-1> minor_matrix(int row, int col)
    {
        int colCount = 0, rowCount = 0;
        Matrix<N-1> ret;
        for(int i = 0; i < N; i++ )
        {
            if( i != row )
            {
                for(int j = 0; j < N; j++ )
                {
                    if( j != col )
                    {
                        ret(rowCount, colCount) = cell(i, j);
                        colCount++;
                    }
                }
                rowCount++;
            }
        }
        return ret;
    }

    double determinant()
    {
        if(N == 1)
            return cell(0, 0);

        float det = 0.0;
        for(int i = 0; i < N; i++ )
        {
            Matrix<N-1> minor = minor_matrix(0, i);
            det += (i%2==1?-1.0:1.0) * cell(0, i) * minor.determinant();
        }
        return det;
    }

    Matrix invert()
    {
        Matrix ret;
        float det = determinant();

        for(int x = 0; x < N; x++)
        {
            for(int y = 0; y < N; y++)
            {
                Matrix<N-1> minor = minor_matrix(y, x);
                ret(x, y) = det*minor.determinant();
                if( (x+y)%2 == 1)
                    ret(x, y) = -ret(x, y);
            }
        }
        return ret;
    }

private:
    double* _cell;
    double  _cell_data[N*N];
};


};

#endif

