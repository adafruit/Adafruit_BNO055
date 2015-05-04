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

namespace imu
{


template <uint8_t N> class Matrix
{
public:
    Matrix()
    {
        int r = sizeof(double)*N;
        _cell = &_cell_data[0];
        memset(_cell, 0, r*r);
    }

    Matrix(const Matrix &v)
    {
        int r = sizeof(double)*N;
        _cell = &_cell_data[0];
        memset(_cell, 0, r*r);
        for (int x = 0; x < N; x++ )
        {
            for(int y = 0; y < N; y++)
            {
                _cell[x*N+y] = v._cell[x*N+y];
            }
        }
    }

    ~Matrix()
    {
    }

    void operator = (Matrix m)
    {
        for(int x = 0; x < N; x++)
        {
            for(int y = 0; y < N; y++)
            {
                cell(x, y) = m.cell(x, y);
            }
        }
    }

    Vector<N> row_to_vector(int y)
    {
        Vector<N> ret;
        for(int i = 0; i < N; i++)
        {
            ret[i] = _cell[y*N+i];
        }
        return ret;
    }

    Vector<N> col_to_vector(int x)
    {
        Vector<N> ret;
        for(int i = 0; i < N; i++)
        {
            ret[i] = _cell[i*N+x];
        }
        return ret;
    }

    void vector_to_row(Vector<N> v, int row)
    {
        for(int i = 0; i < N; i++)
        {
            cell(row, i) = v(i);
        }
    }

    void vector_to_col(Vector<N> v, int col)
    {
        for(int i = 0; i < N; i++)
        {
            cell(i, col) = v(i);
        }
    }

    double& operator ()(int x, int y)
    {
        return _cell[x*N+y];
    }

    double& cell(int x, int y)
    {
        return _cell[x*N+y];
    }


    Matrix operator + (Matrix m)
    {
        Matrix ret;
        for(int x = 0; x < N; x++)
        {
            for(int y = 0; y < N; y++)
            {
                ret._cell[x*N+y] = _cell[x*N+y] + m._cell[x*N+y];
            }
        }
        return ret;
    }

    Matrix operator - (Matrix m)
    {
        Matrix ret;
        for(int x = 0; x < N; x++)
        {
            for(int y = 0; y < N; y++)
            {
                ret._cell[x*N+y] = _cell[x*N+y] - m._cell[x*N+y];
            }
        }
        return ret;
    }

    Matrix operator * (double scalar)
    {
        Matrix ret;
        for(int x = 0; x < N; x++)
        {
            for(int y = 0; y < N; y++)
            {
                ret._cell[x*N+y] = _cell[x*N+y] * scalar;
            }
        }
        return ret;
    }

    Matrix operator * (Matrix m)
    {
        Matrix ret;
        for(int x = 0; x < N; x++)
        {
            for(int y = 0; y < N; y++)
            {
                Vector<N> row = row_to_vector(x);
                Vector<N> col = m.col_to_vector(y);
                ret.cell(x, y) = row.dot(col);
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

