///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Neural Tissue
//  Subsystem:       Math
//  Workfile:        MatrixMath.cpp
//  Revision:        1.0 - 11 March, 2009
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Collection of math functions applicable to numerical matrices
//
///////////////////////////////////////////////////////////////////////////////

#include "MatrixMath.h"
#include "math.h"

namespace Math
{
  LIBRARY_API bool matrixMult (matrix &val1, matrix &val2, matrix &out)
  {
    int m1, m2, n1, n2, x2, y1, z;
    double sum;
    m1 = val1.rows;
    m2 = val2.rows;

    if (m1 < 1 || m2 < 1)
    {
      return false;
    }

    n1 = val1.cols;
    n2 = val2.cols;

    if (n1 < 1 || n2 < 1 || n1 != m2)
    {
      return false;
    }

//    out.resize (m1, n2);

    for (y1 = 0; y1 < m1; ++y1)
    {
      for (x2 = 0; x2 < n2; ++x2)
      {
        sum = 0.0f;
        for (z = 0; z < n1; ++z)
        {
          sum += val1.at(y1, z) * val2.at(z, x2);
        }
        out.at(y1, x2) = sum;
      }
    }
    return true;
  }


  LIBRARY_API bool matrixMult (matrix &val1, double &mult, matrix &out)
  {
    int m1, n1, x1, y1;
    m1 = val1.rows;
    if (m1 < 1)
    {
      return false;
    }

    n1 = val1.cols;
    if (n1 < 1)
    {
      return false;
    }

//    out.resize(m1, n1);

    for (y1 = 0; y1 < m1; ++y1)
    {
      for (x1 = 0; x1 < n1; ++x1)
      {
        out.at(y1, x1) = val1.at(y1, x1) * mult;
      }
    }

    return true;
  }


  LIBRARY_API bool matrixAdd (matrix &val1, matrix &val2, matrix &out)
  {
    int m1, m2, n1, n2, x1, y1;
    m1 = val1.rows;
    m2 = val2.rows;

    if (m1 < 1 || m2 < 1 || m1 != m2)
    {
      return false;
    }

    n1 = val1.cols;
    n2 = val1.cols;

    if (n1 < 1 || n2 < 1 || n1 != n2)
    {
      return false;
    }

//    out.resize (m1, n1);

    for (y1 = 0; y1 < m1; ++y1)
    {
      for (x1 = 0; x1 < n1; ++x1)
      {
        out.at(y1, x1) = val1.at(y1, x1) + val2.at(y1, x1);
      }
    }

    return true;
  }


  LIBRARY_API bool matrixTrans (matrix &val1, matrix &out)
  {
    int m1, n1, x1, y1;
    double val;
    m1 = val1.rows;
    if (m1 < 1)
    {
      return false;
    }

    n1 = val1.cols;
    if (n1 < 1)
    {
      return false;
    }

//    out.resize (n1, m1);

    for (y1 = 0; y1 < m1; ++y1)
    {
      for (x1 = 0; x1 < n1; ++x1)
      {
        val = val1.at(y1, x1);
        out.at(x1, y1) = val;
      }
    }

    return true;
  }


  //! Based on gaussj from "Numerical Recipes in C"
  //!
  // FMP moved before matrixPseudoInv() which calls this
  LIBRARY_API bool matrixInv (matrix &val1, matrix &out)
  {
    int *indxc, *indxr, *ipiv;
    int i, icol, irow, j, k, l, ll;
    int x, y, n = val1.rows;
    double big, dum, pivinv, temp;

    if (val1.rows < 1 || val1.rows != val1.cols)
    {
      return false;
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
        temp = val1.at(y, x);
        out.at(y, x) = temp;
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
             if (fabs(out.at(j, k)) >= big)
             {
               big = fabs(out.at(j, k));
               irow = j;
               icol = k;
             }      
           } // if (ipiv[k] == 0)
           else if (ipiv[k] > 1)
           {
  //           nrerror("GAUSSJ: Singular Matrix-1");
           }
         } // for (k=1;k<=n;k++)
       } // if (ipiv[j] != 1)
      } // for (j=1;j<=n;j++)

      ++(ipiv[icol]);
      if (irow != icol)
      {
        for (l = 0; l < n; ++l)
        {
          temp = out.at(irow, l);
          out.at(irow, l) = out.at(icol, l);
          out.at(icol, l) = temp;
        }
      }

      indxr[i] = irow;
      indxc[i] = icol;
      temp = fabs(out.at(icol, icol));
      if (temp < 0.00000001)
      {
//        exception ("matrixInv", "Singular Matrix:2");
      }

      pivinv = 1.0 / out.at(icol, icol);
      out.at(icol, icol) = 1.0;

      for (l = 0; l < n; ++l)
      {
        temp = out.at(icol, l) * pivinv;
        out.at(icol, l) = temp;
      }

      for (ll = 0; ll < n; ++ll)
      {
        if (ll != icol) 
        {
          dum = out.at(ll, icol);
          out.at(ll, icol) = 0.0f;

          for (l = 0; l < n; ++l)
          {
            temp = out.at (ll, l) - (out.at(icol, l) * dum);
            out.at(ll, l) = temp;
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
          temp = out.at(k, indxr[l]);
          out.at(k, indxr[l]) = out.at(k, indxc[l]);
          out.at(k, indxc[l]) = temp;
        }
      }
    }

    delete [] ipiv;
    delete [] indxr;
    delete [] indxc;

    return true;
  }


  LIBRARY_API bool matrixPseudoInv (matrix &in, matrix &out)
  {
    matrix in_T, inTin, inTinInv;
    bool state;

    state = matrixTrans (in, in_T);

    state = state ? matrixMult (in_T, in, inTin) : state;
    state = state ? matrixInv (inTin, inTinInv) : state;
    state = state ? matrixMult (inTinInv, in_T, out) : state;
    return state;
  }


  LIBRARY_API bool rotMatrixEulerConvert (matrix &m, vector<double> &out)
  {
    double xr = 0.0f;
    double yr = 0.0f;
    double zr = 0.0f;

    if (m.cols != 3 || m.rows != 3)
    {
      return false;
    }

    out.clear();

    yr = atan2(-(m.at(2, 0)), sqrt((m.at(0, 0) * m.at(0, 0)) + (m.at(1, 0) * m.at(1, 0))));

    if (fabs(yr - 1.57079632679489661923f) < 1.0e-4)
    {
      xr = atan2 (m.at(0, 1), m.at(1, 1));
      yr = 1.57079632679489661923f;
      zr = 0.0f;
    }
    else if (fabs(yr + 1.57079632679489661923f) < 1.0e-4)
    {
      xr = -atan2(m.at(0, 1), m.at(1, 1));
      yr = -1.57079632679489661923f;
      zr = 0.0f;
    }
    else
    {
      xr = atan2(m.at(2, 1), m.at(2, 2));
      zr = atan2(m.at(1, 0), m.at(0, 0));
    }

    out.push_back(xr);
    out.push_back(yr);
    out.push_back(zr);

    return true;
  }
 

  LIBRARY_API bool rotEulerMatrixConvert (vector<double> &v, matrix &out)
  {
    double sa, sb, sg;
    double ca, cb, cg;

    if (v.size() != 3 || out.cols != 3 || out.rows != 3)
    {
      return false;
    }

    sa = sin(v.at(2));
    sb = sin(v.at(1));
    sg = sin(v.at(0));

    ca = cos(v.at(2));
    cb = cos(v.at(1));
    cg = cos(v.at(0));

    out.at(0, 0) = ca * cb;
    out.at(0, 1) = ca * sb * sg - sa * cg;
    out.at(0, 2) = ca * sb * cg + sa * sg;

    out.at(1, 0) = sa * cb;
    out.at(1, 1) = sa * sb * sg + ca * cg;
    out.at(1, 2) = sa * sb * cg - ca * sg;

    out.at(2, 0) = -sb;
    out.at(2, 1) = cb * sg;
    out.at(2, 2) = cb * cg;

    return true;
  }


  LIBRARY_API bool rotMatrixAxisAngleConvert (matrix &m, vector<double> &out)
  {
    double angle = acos((m.at(0, 0) + m.at(1, 1) + m.at(2, 2) - 1.0f) / 2.0f);
    double v1 = m.at(2,1) - m.at(1,2),
           v2 = m.at(0,2) - m.at(2,0),
           v3 = m.at(1,0) - m.at(0,1);
    double div = sqrt((v1 * v1) + (v2 * v2) + (v3 * v3));
    double x = (m.at(2, 1) - m.at(1, 2)) / div,
           y = (m.at(0, 2) - m.at(2, 0)) / div,
           z = (m.at(1, 0) - m.at(0, 1)) / div;

    if (m.cols != 3 || m.rows != 3)
    {
      return false;
    }

    out.clear();

    out.push_back(x);
    out.push_back(y);
    out.push_back(z);

    return true;
  }
 

  LIBRARY_API bool rotAxisAngleMatrixConvert (vector<double> &v, matrix &out)
  {
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
    
    out.at(0,0) = (x * x * bigC) + c;
    out.at(0,1) = (x * y * bigC) - (z * s);
    out.at(0,2) = (x * z * bigC) + (y * s);
    out.at(1,0) = (y * x * bigC) + (z * s);
    out.at(1,1) = (y * y * bigC) + c;
    out.at(1,2) = (y * z * bigC) - (x * s);
    out.at(2,0) = (z * x * bigC) - (y * s);
    out.at(2,1) = (z * y * bigC) + (x * s);
    out.at(2,2) = (z * z * bigC) + c;

    return true;
  }

}
