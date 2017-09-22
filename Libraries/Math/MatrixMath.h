///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Math
//  Workfile:        MatrixMath.h
//  Revision:        1.0 - 11 March, 2009
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Collection of matrix-based mathematical functions
//
///////////////////////////////////////////////////////////////////////////////

#ifndef MATRIX_MATH_H
#define MATRIX_MATH_H

#include <vector>
#include "VectorMath.h"
#pragma warning (disable: 4018)

using namespace std;

namespace Math
{

  struct matrix
  {
    //! @brief The matrix values
    //!
    double **data;
  
    //! @brief The number of rows in the matrix
    //!
    int rows;

    //! @brief The number of columns in the matrix
    //!
    int cols;

    //! @brief Whether or not the current matrix is considered valid
    //!
    bool valid;

    //! @brief Default constructor
    //!
    matrix ()
    {
      rows = cols = 0;
      data = NULL;
      valid = false;
    }


    //! @brief Copy constructor
    //!
    matrix (const matrix& source)
    {
      rows = source.rows;
      cols = source.cols;
      int x, y;
      valid = true;

      data = new double*[rows];
      for (y = 0; y < rows; ++y)
      {
        data[y] = new double[cols];
        for (x = 0; x < cols; ++x)
        {
          data[y][x] = source.data[y][x];
        }
      }
    }


    //! @brief Resize constructor
    //!
    //! @param r The number of rows in the new matrix
    //! @param c The number of columns in the new matrix
    //!
    matrix (int r, int c)
    {
      int y, x;
      rows = r;
      cols = c;
      valid = true;

      data = new double*[rows];
      for (y = 0; y < rows; ++y)
      {
        data[y] = new double[cols];
        for (x = 0; x < cols; ++x)
        {
          data[y][x] = 0.0f;
        }
      }
    }


    //! @brief Default destructor
    //!
    ~matrix ()
    {
      int y;
      if (data != NULL && rows > 0 && cols > 0)
      {
        for (y = 0; y < rows; ++y)
        {
          if (data[y] != NULL)
          {
            delete data[y];
          }
        }
        delete data;
        data = NULL;
      }
      rows = cols = 0;
      valid = false;
    }


    //! @brief Resize the matrix (note that this deletes the current matrix values)
    //!
    //! @param r The number of rows in the resized matrix
    //! @param c The number of columns in the resized matrix
    //!
    void resize (int r, int c)
    {
      int x, y;
      if (data != NULL)
      {
        for (y = 0; y < rows; ++y) 
        {
          delete [] data[y];
        }
        delete [] data;
      }

      rows = r;
      cols = c;
      data = new double*[rows];
      for (y = 0; y < rows; ++y)
      {
        data[y] = new double[cols];
        for (x = 0; x < cols; ++x)
        {
          data[y][x] = 0.0f;
        }
      }
      valid = true;
    }


    //! @brief Data accessor
    //!
    //! @param row The row of the matrix to access
    //! @param col The column of the matrix to access
    //!
    //! @return A pointer to the matrix element specified
    //!
    //! @note:  This function does not verify that row and col are valid values
    //!
    double& at(int row, int col)
    {
      return data[row][col];
    }


    //! @brief Assign all elements in the matrix to be a specified value
    //!
    //! @param val The value to which all elements of the matrix will be set
    //!
    void setAll(double val)
    {
      valid = true;
      int x, y;

      for (y = 0; y < rows; ++y)
      {
        for (x = 0; x < cols; ++x)
        {
          data[y][x] = val;
        }
      }
    }


    //! @brief Generate a covariance matrix from two vectors of random varialbes
    //!
    //! @param v1 Input vector #1
    //! @param v2 Input vector #2
    //!
    //! @return True if the covariance matrix was created successfully, false otherwise
    //!
    bool covariance(vector<double> &v1, vector<double> &v2)
    {
      double avgV1 = 0.0f,
             avgV2 = 0.0f,
             sum;
      int i, j, k, sz;

      if (v1.size() != v2.size())
      {
        valid = false;
        return false;
      }
      sz = v1.size();

      resize(sz, sz);
      matrix m1(sz, 1);
      matrix m2(1, sz);

      for (i = 0; i < sz; ++i)
      {
        avgV1 += v1.at(i);
        avgV2 += v2.at(i);
      }
      avgV1 /= sz;
      avgV2 /= sz;

      for (i = 0; i < sz; ++i)
      {
        m1.at(i, 0) = v1.at(i) - avgV1;
        m2.at(0, i) = v2.at(i) - avgV2;
      }

      for (i = 0; i < sz; ++i)
      {
        for (j = 0; j < sz; ++j)
        {
          sum = 0.0f;
          for (k = 0; k < 1; ++k)
          {
            sum += m1.at(i, k) * m2.at(k, j);
          } // for (k = 0; k < 1; ++k)
          data[i][j] = sum;
        } // for (i = 0; j < sz; ++j)
      } // for (i = 0; i < sz; ++i)
      valid = true;
      return true;
    }


    //! @brief Create a 4-element column matrix from a source point object
    //!
    //! @param source The source point object from which a column matrix is created
    //!
    void homogeneousPoint(const point &source)
    {
      resize(4, 1);
      data[0][0] = source.x;
      data[1][0] = source.y;
      data[2][0] = source.z;
      data[3][0] = 1.0f;
      valid = true;
    }


    //! @brief Create an NxN identitity matrix
    //!
    //! @param dim The new dimensions (dim x dim) of the identity matrix.  If dim < 1 a 1x1 matrix is
    //             generated, instead;
    //!
    void identity(int dim)
    {
      int sz = (dim < 1) ? 1 : dim;
      resize(sz, sz);
      setAll (0.0f);
      for (int i = 0; i < sz; ++i)
      {
        at(i, i) = 1.0f;
      }
      valid = true;
    }


    //! @brief Matrix assignment function
    //!
    //! @param source An existing matrix that will be used to populate this matrix
    //!               instance
    //!
    matrix & operator=(const matrix &source)
    {
      if (this != &source)
      {
        resize(source.rows, source.cols);
        for (int i = 0; i < rows; ++i)
        {
          for (int j = 0; j < cols; ++j)
          {
            data[i][j] = source.data[i][j];
          }
        }
      }
      valid = true;
      return *this;
    }


    //! @brief Matrix assignment function from a point (creates a 3x1 matrix)
    //!
    //! @param source A 3D point object that will be used to populate this matrix
    //!               instance
    //!
    matrix & operator=(const point &source)
    {
      resize (3, 1);
      data[0][0] = source.x;
      data[1][0] = source.y;
      data[2][0] = source.z;
      valid = true;
      return *this;
    }


    //! @brief Matrix assignment function from a vector (creates a Nx1 matrix)
    //!
    //! @param source A vector of length N that will be used to populate this matrix
    //!               instance
    //!
    matrix & operator=(const vector<double, allocator<double> > &source)
    {
      resize(source.size(), 1);
      for (int x = 0; x < source.size(); ++x)
      {
        data[x][0] = source.at(x);
      }
      valid = true;
      return *this;
    }


    //! @brief Matrix-matrix multiplication operator
    //!
    //! @param val Multiplier matrix
    //!
    //! @return The product of the two matrices
    //!
    matrix operator*(const matrix& val)
    {
      int m1, m2, n1, n2, x2, y1, z;
      matrix out;
      out.resize(rows, val.cols);
      double sum;
      m1 = rows;
      m2 = val.rows;

      if (m1 < 1 || m2 < 1)
      {
        out.valid = false;
        return out;
      }

      n1 = cols;
      n2 = val.cols;

      if (n1 < 1 || n2 < 1 || n1 != m2)
      {
        out.valid = false;
        return out;
      }

      for (y1 = 0; y1 < m1; ++y1)
      {
        for (x2 = 0; x2 < n2; ++x2)
        {
          sum = 0.0f;
          for (z = 0; z < n1; ++z)
          {
            sum += data[y1][z] * val.data[z][x2];
          } // for (z = 0; z < n1; ++z)
          out.data[y1][x2] = sum; //at(y1, x2) = sum;
        } // for (x2 = 0; x2 < n2; ++x2)
      } // for (y1 = 0; y1 < m1; ++y1)
      out.valid = true;
      return out;
    }


    //! @brief Matrix-scalar multiplier operator
    //!
    //! @param val The constant value by which the matrix is scaled
    //!
    //! @return The scaled matrix
    //!
    matrix operator*(double& val)
    {
      int m1, n1, x1, y1;
      matrix out;
      out.resize(rows, cols);
      m1 = rows;
      if (m1 < 1)
      {
        out.valid = false;
        return out;
      }

      n1 = cols;
      if (n1 < 1)
      {
        out.valid = false;
        return out;
      }

      for (y1 = 0; y1 < m1; ++y1)
      {
        for (x1 = 0; x1 < n1; ++x1)
        {
          out.at(y1, x1) = data[y1][x1] * val;
        }
      }

      out.valid = true;
      return out;
    }


    //! @brief Matrix-scalar multiplier operator
    //!
    //! @param val The constant value by which the matrix is scaled
    //!
    //! @return The scaled matrix
    //!
    matrix operator*(const double val)
    {
      int m1, n1, x1, y1;
      matrix out;
      out.resize(rows, cols);
      m1 = rows;
      if (m1 < 1)
      {
        out.valid = false;
        return out;
      }

      n1 = cols;
      if (n1 < 1)
      {
        out.valid = false;
        return out;
      }

      for (y1 = 0; y1 < m1; ++y1)
      {
        for (x1 = 0; x1 < n1; ++x1)
        {
          out.at(y1, x1) = data[y1][x1] * val;
        }
      }

      out.valid = true;
      return out;
    }


    //! @brief Matrix-scalar divisor operator
    //!
    //! @param val The constant value by which the matrix is scaled
    //!
    //! @return The scaled matrix
    //!
    matrix operator/(double& val)
    {
      int m1, n1, x1, y1;
      matrix out;
      out.resize(rows, cols);
      m1 = rows;
      if (m1 < 1)
      {
        out.valid = false;
        return out;
      }

      n1 = cols;
      if (n1 < 1)
      {
        out.valid = false;
        return out;
      }

      for (y1 = 0; y1 < m1; ++y1)
      {
        for (x1 = 0; x1 < n1; ++x1)
        {
          out.at(y1, x1) = data[y1][x1] / val;
        }
      }

      out.valid = true;
      return out;
    }


    //! @brief Matrix-scalar divisor operator
    //!
    //! @param val The constant value by which the matrix is scaled
    //!
    //! @return The scaled matrix
    //!
    matrix operator/(const double val)
    {
      int m1, n1, x1, y1;
      matrix out;
      out.resize(rows, cols);
      m1 = rows;
      if (m1 < 1)
      {
        out.valid = false;
        return out;
      }

      n1 = cols;
      if (n1 < 1)
      {
        out.valid = false;
        return out;
      }

      for (y1 = 0; y1 < m1; ++y1)
      {
        for (x1 = 0; x1 < n1; ++x1)
        {
          out.at(y1, x1) = data[y1][x1] / val;
        }
      }

      out.valid = true;
      return out;
    }


    //! @brief Matrix-matrix addition operator
    //!
    //! @param val Additive matrix
    //!
    //! @return The sum of the two matrices
    //!
    matrix operator+(const matrix &val)
    {
      int m1, m2, n1, n2, x1, y1;
      matrix out (rows, cols);
      m1 = rows;
      m2 = val.rows;

      if (m1 < 1 || m2 < 1 || m1 != m2)
      {
        out.valid = false;
        return out;
      }

      n1 = cols;
      n2 = val.cols;

      if (n1 < 1 || n2 < 1 || n1 != n2)
      {
        out.valid = false;
        return out;
      }

      for (y1 = 0; y1 < m1; ++y1)
      {
        for (x1 = 0; x1 < n1; ++x1)
        {
          out.data[y1][x1] = data[y1][x1] + val.data[y1][x1];
        }
      }

      out.valid = true;
      return out;
    }


    //! @brief Matrix-matrix subtraction operator
    //!
    //! @param val Subtractive matrix
    //!
    //! @return The differences of the two matrices
    //!
    matrix operator-(const matrix &val)
    {
      int m1, m2, n1, n2, x1, y1;
      matrix out;
      out.resize(rows, cols);
      m1 = rows;
      m2 = val.rows;

      if (m1 < 1 || m2 < 1 || m1 != m2)
      {
        out.valid = false;
        return out;
      }

      n1 = cols;
      n2 = val.cols;

      if (n1 < 1 || n2 < 1 || n1 != n2)
      {
        out.valid = false;
        return out;
      }

      for (y1 = 0; y1 < m1; ++y1)
      {
        for (x1 = 0; x1 < n1; ++x1)
        {
          out.at(y1, x1) = data[y1][x1] - val.data[y1][x1];
        }
      }

      out.valid = true;
      return out;
    }


    //! @brief Produce the transpose of the matrix
    //!
    matrix trans ()
    {
      int m1, n1, x1, y1;
      double val;
      matrix out;
      out.resize(cols, rows);
      m1 = rows;
      if (m1 < 1)
      {
        out.valid = false;
        return out;
      }

      n1 = cols;
      if (n1 < 1)
      {
        out.valid = false;
        return out;
      }

      for (y1 = 0; y1 < m1; ++y1)
      {
        for (x1 = 0; x1 < n1; ++x1)
        {
          val = data[y1][x1];
          out.at(x1, y1) = val;
        }
      }

      out.valid = true;
      return out;
    }


    //! @brief Produce the inverse of the matrix
    //!
    //! @return The inverse of the current matrix (if it exists)
    //!
    //! @note:  Based on gaussj from "Numerical Recipes in C"
    //!
    matrix inv()
    {
      matrix out;
      out.resize(rows, cols);
      int *indxc, *indxr, *ipiv;
      int i, icol, irow, j, k, l, ll;
      int x, y, n = rows;
      double big, dum, pivinv, temp;

      if (rows < 1 || rows != cols)
      {
        out.valid = false;
        return out;
      }

      //! Index and pivot tables
      indxc = new int[n]; //ivector(1,n);
      indxr = new int[n]; //indxr=ivector(1,n);
      ipiv = new int[n]; //ipiv=ivector(1,n);

      //! Create a copy of our input vector for in-line inversion
      for (y = 0; y < n; ++y)
      {
        for (x = 0; x < n; ++x)
        {
          temp = data[y][x];
          out.data[y][x] = temp;
        }
      }

      for (j = 0; j < n; ++j)
      {
        ipiv[j] = 0;
      }

      for (i = 0; i < n; ++i)
      {
        big = 0.0f;
        for (j = 0; j < n; ++j)
        {
          if (ipiv[j] != 1)
          {
            for (k = 0;k < n; ++k)
            {
              if (ipiv[k] == 0)
              {
                if (fabs(out.data[j][k]) >= big)
                {
                  big = fabs(out.data[j][k]);
                  irow = j;
                  icol = k;
                }      
              } // if (ipiv[k] == 0)
              else if (ipiv[k] > 1)
              {
                 out.valid = false;
                 return out;
              }
            } // for (k=1;k<=n;k++)
          } // if (ipiv[j] != 1)
        } // for (j=1;j<=n;j++)

        ++(ipiv[icol]);
        if (irow != icol)
        {
          for (l = 0; l < n; ++l)
          {
            temp = out.data[irow][l];
            out.data[irow][l] = out.data[icol][l];
            out.data[icol][l] = temp;
          }
        }

        indxr[i] = irow;
        indxc[i] = icol;
        temp = fabs(out.data[icol][icol]);
        if (temp < 0.00000001)
        {
           out.valid = false;
           return out;
        }

        pivinv = 1.0 / out.data[icol][icol];
        out.data[icol][icol] = 1.0;

        for (l = 0; l < n; ++l)
        {
          temp = out.data[icol][l] * pivinv;
          out.data[icol][l] = temp;
        }

        for (ll = 0; ll < n; ++ll)
        {
          if (ll != icol) 
          {
            dum = out.data[ll][icol];
            out.data[ll][icol] = 0.0f;

            for (l = 0; l < n; ++l)
            {
              temp = out.data[ll][l] - (out.data[icol][l] * dum);
              out.data[ll][l] = temp;
            }
          }
        } // for (ll = 0; ll < n; ++ll)
      } // for (i = 0; i < n; ++i)

      for (l = n-1; l >= 0; --l)
      {
        if (indxr[l] != indxc[l])
        {
          for (k = 0; k < n; ++k)
          {
            temp = out.data[k][indxr[l]];
            out.data[k][indxr[l]] = out.data[k][indxc[l]];
            out.data[k][indxc[l]] = temp;
          }
        } // if (indxr[l] != indxc[l])
      } //for (l = n-1; l >= 0; --l)

      //! Garbage collection
      delete [] ipiv;
      delete [] indxr;
      delete [] indxc;

      out.valid = true;
      return out;
    }

   
    //! @brief Compute the pseudo inverse of the matrix
    //!
    //! @return A matrix containing the pseudo inverse of the original matrix
    //!
    matrix pseudoInv ()
    {
      matrix in_T, inTin, inTinInv;
      matrix out;
      in_T = this->trans();
      if (!in_T.valid)
      {
        out.valid = false;
        return out;
      }
      inTin = in_T * (*this);
      if (!inTin.valid)
      {
        out.valid = false;
        return out;
      }
      inTinInv = inTin.inv();
      if (!inTinInv.valid)
      {
        out.valid = false;
        return out;
      }
      out = inTinInv * in_T;
      return out;
    }


    //! @brief Create Euler angle representation of an input rotation matrix
    //!
    //! @param out The resultant 3-element vector containing the Euler angles (xr, yr, zr)
    //!
    //! @return True if the function completed successfully, False otherwise
    //!
    //! @note Principal check is for the origin matrix to be 3x3
    //!
    bool rotMatrixEulerConvert (vector<double> &out)
    {
      double xr = 0.0f;
      double yr = 0.0f;
      double zr = 0.0f;

      if (cols != 3 || rows != 3)
      {
        return false;
      }

      out.clear();

      yr = atan2(-(at(2, 0)), sqrt((at(0, 0) * at(0, 0)) + (at(1, 0) * at(1, 0))));

      if (fabs(yr - 1.57079632679489661923f) < 1.0e-4)
      {
        xr = atan2 (at(0, 1), at(1, 1));
        yr = 1.57079632679489661923f;
        zr = 0.0f;
      }
      else if (fabs(yr + 1.57079632679489661923f) < 1.0e-4)
      {
        xr = -atan2(at(0, 1), at(1, 1));
        yr = -1.57079632679489661923f;
        zr = 0.0f;
      }
      else
      {
        xr = atan2(at(2, 1), at(2, 2));
        zr = atan2(at(1, 0), at(0, 0));
      }

      out.push_back(xr);
      out.push_back(yr);
      out.push_back(zr);

      return true;
    }
 

    //! @brief Create rotation matrix based on an input Euler angle rotation vector
    //!
    //! @param v Input 3x1 Euler rotation vector
    //!
    //! @return True if the conversion completed successfully, False otherwise
    //!
    //! @note Overwrites the current value of the origin matrix
    //!
    bool rotEulerMatrixConvert (vector<double> &v)
    {
      double sa, sb, sg;
      double ca, cb, cg;

      if (v.size() != 3 || cols != 3 || rows != 3)
      {
        return false;
      }

      sa = sin(v.at(2));
      sb = sin(v.at(1));
      sg = sin(v.at(0));

      ca = cos(v.at(2));
      cb = cos(v.at(1));
      cg = cos(v.at(0));

      at(0, 0) = ca * cb;
      at(0, 1) = ca * sb * sg - sa * cg;
      at(0, 2) = ca * sb * cg + sa * sg;

      at(1, 0) = sa * cb;
      at(1, 1) = sa * sb * sg + ca * cg;
      at(1, 2) = sa * sb * cg - ca * sg;

      at(2, 0) = -sb;
      at(2, 1) = cb * sg;
      at(2, 2) = cb * cg;

      return true;
    }


    //! @brief Create an axis-angle representation of an input rotation matrix
    //!
    //! @param out The output 3x1 (xr, yr, zr) axis-axis angle vector, where the vector
    //!            is the axis, and magnitude of the vector is the value of the angle
    //!
    //! @return True if the conversion completed successfully, False otherwise
    //!
    bool rotMatrixAxisAngleConvert (vector<double> &out)
    {
      double angle = acos((at(0, 0) + at(1, 1) + at(2, 2) - 1.0f) / 2.0f);
      double v1 = at(2,1) - at(1,2),
             v2 = at(0,2) - at(2,0),
             v3 = at(1,0) - at(0,1);
      double div = sqrt((v1 * v1) + (v2 * v2) + (v3 * v3));
      double x = (at(2, 1) - at(1, 2)) / div,
             y = (at(0, 2) - at(2, 0)) / div,
             z = (at(1, 0) - at(0, 1)) / div;

      if (cols != 3 || rows != 3)
      {
        return false;
      }

      out.clear();

      out.push_back(x * angle);
      out.push_back(y * angle);
      out.push_back(z * angle);

      return true;
    }


    //! @brief Create a rotation matrix based on an input axis-angle representation vector
    //!
    //! @param v The input axis-angle vector, where v is the axis, and |v| is the angle
    //!
    //! @return True if the conversion completed successfully, False otherwise
    //!
    //! @note Overwrites the current value of the origin matrix if the conversion is successful
    //!
    bool rotAxisAngleMatrixConvert (vector<double> &v)
    {
      if (rows != 3 || cols != 3)
      {
        return false;
      }

      double x = v.at(0), 
             y = v.at(1),
             z = v.at(2);

      double d = sqrt((x * x) + (y * y) + (z * z));
      x /= d;
      y /= d;
      z /= d;

      double c = cos (d),
             s = sin (d),
             bigC = 1.0f - c;
    
      at(0,0) = (x * x * bigC) + c;
      at(0,1) = (x * y * bigC) - (z * s);
      at(0,2) = (x * z * bigC) + (y * s);
      at(1,0) = (y * x * bigC) + (z * s);
      at(1,1) = (y * y * bigC) + c;
      at(1,2) = (y * z * bigC) - (x * s);
      at(2,0) = (z * x * bigC) - (y * s);
      at(2,1) = (z * y * bigC) + (x * s);
      at(2,2) = (z * z * bigC) + c;

      return true;
    }


    //! @brief Create a quaternion representation of a rotation matrix
    //!
    //! @param out The resultant quaternion (w, x, y, z) from the conversion
    //!
    //! @return True if the conversion completes successfully, False otherwise
    //!
    bool rotMatrixQuaternionConvert (vector<double> &out)
    {
      double qw, qx, qy, qz;
      qw = qx = qy = qz = 0.0f;

      if (cols != 3 || rows != 3)
      {
        return false;
      }

      qw = sqrt(1.0f + at(0, 0) + at(1, 1) + at(2, 2)) / 2.0f;
      qx = (at(2, 1) - at(1, 2)) / (4.0f * qw);
      qy = (at(0, 2) - at(2, 0)) / (4.0f * qw);
      qz = (at(1, 0) - at(0, 1)) / (4.0f * qw);

      out.clear();

      out.push_back(qw);
      out.push_back(qx);
      out.push_back(qy);
      out.push_back(qz);

      return true;
    }


    //! @brief Create a rotation matrix from an input quaternion vector
    //!
    //! @param v The input quaternion (w, x, y, z) to be converted
    //!
    //! @return True if the conversion completes successfully, False otherwise
    //!
    //! @note Overwrites the origin matrix if the conversion is successful
    //!
    bool rotQuaternionMatrixConvert (vector<double> &v)
    {
      if (rows != 3 || cols != 3)
      {
        return false;
      }

      if (v.size() != 4)
      {
        return false;
      }

      double w = v.at(0), 
             x = v.at(1),
             y = v.at(2),
             z = v.at(3);
      double sqw = w*w,
             sqx = x*x,
             sqy = y*y,
             sqz = z*z;
      double inv = 1.0f / (sqw + sqx + sqy + sqz); //! Needed only if quaternion not normalized
      at(0, 0) = (sqx - sqy - sqz + sqw) * inv;
      at(1, 1) = (-sqx + sqy - sqz + sqw) * inv;
      at(2, 2) = (-sqx - sqy + sqz + sqw) * inv;

      double tmp1 = x * y,
             tmp2 = z * w;
      at(1, 0) = 2.0f * (tmp1 + tmp2) * inv;
      at(0, 1) = 2.0f * (tmp1 - tmp2) * inv;

      tmp1 = x * z;
      tmp2 = y * w;
      at(2, 0) = 2.0f * (tmp1 - tmp2) * inv;
      at(0, 2) = 2.0f * (tmp1 + tmp2) * inv;

      tmp1 = y * z;
      tmp2 = x * w;
      at(2, 1) = 2.0f * (tmp1 + tmp2) * inv;
      at(1, 2) = 2.0f * (tmp1 - tmp2) * inv;

      return true;
    } // bool rotQuaternionMatrixConvert (vector<double> &v)


      //! JAM: these should be moved to the math library
    bool matrixRPYConvert(pose &poseOut, bool useDegrees)
    {
      if (rows != 4 || cols != 4)
      {
        return false;
      }

      poseOut.yr = atan2(-(at(2, 0)), sqrt((at(0, 0) * at(0, 0)) + (at(1, 0) * at(1, 0))));

      if (fabs(poseOut.yr - 1.57079632679489661923f) < 1.0e-4)
      {
        poseOut.xr = atan2(at(0, 1), at(1, 1));
        poseOut.yr = 1.57079632679489661923f;
        poseOut.zr = 0.0f;
      }
      else if (fabs(poseOut.yr + 1.57079632679489661923f) < 1.0e-4)
      {
        poseOut.xr = -atan2(at(0, 1), at(1, 1));
        poseOut.yr = -1.57079632679489661923f;
        poseOut.zr = 0.0f;
      }
      else
      {
        poseOut.xr = atan2(at(2, 1), at(2, 2));
        poseOut.zr = atan2(at(1, 0), at(0, 0));
      }

      if (useDegrees)
      {
        poseOut.xr *= (180.0f / 3.141592654);
        poseOut.yr *= (180.0f / 3.141592654);
        poseOut.zr *= (180.0f / 3.141592654);
      }

      if (rows == 4 && cols == 4)
      {
        poseOut.x = at(0, 3);
        poseOut.y = at(1, 3);
        poseOut.z = at(2, 3);
      }

      return true;
    }


    bool RPYMatrixConvert(const pose &poseIn, bool useDegrees)
    {
      double sa, sb, sg;
      double ca, cb, cg;
      pose temp;

      temp = poseIn;

      resize(4, 4);

      if (useDegrees)
      {
        temp.xr *= (3.141592654f / 180.0f);
        temp.yr *= (3.141592654f / 180.0f);
        temp.zr *= (3.141592654f / 180.0f);
      }

      sa = sin(temp.zr);
      sb = sin(temp.yr);
      sg = sin(temp.xr);

      ca = cos(temp.zr);
      cb = cos(temp.yr);
      cg = cos(temp.xr);

      //! JAM TODO:  check for gymbal lock?

      at(0, 0) = ca * cb;
      at(0, 1) = ca * sb * sg - sa * cg;
      at(0, 2) = ca * sb * cg + sa * sg;

      at(1, 0) = sa * cb;
      at(1, 1) = sa * sb * sg + ca * cg;
      at(1, 2) = sa * sb * cg - ca * sg;

      at(2, 0) = -sb;
      at(2, 1) = cb * sg;
      at(2, 2) = cb * cg;

      return true;
    }


    void print()
    {
      for (int i = 0; i < rows; ++i)
      {
        printf ("| ");
        for (int j = 0; j < cols; ++j)
        {
          printf ("%f ", at(i, j));
        }
        printf ("|\n");
      }
    } // void print()
  };
}
#endif