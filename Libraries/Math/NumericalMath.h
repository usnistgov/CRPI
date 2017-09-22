///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Math
//  Workfile:        NumericalMath.h
//  Revision:        1.0 - 21 January, 2008
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Event logging class for reporting errors and program outputs for debugging
//  purposes.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef NUMERICAL_MATH_H
#define NUMERICAL_MATH_H

#define IA 16807
#define IM 2147483647
#define IQ 127773
#define IR 2836
#define NTAB 32
#define EPS (1.2E-07)

#include "../../portable.h"
#include <fstream>

#if defined(_MSC_VER)
#include "math.h"
#include <windows.h>
#elif defined(__GNUC__)
#include <cmath>
#include <stdio.h>
#endif

using namespace std;

///////////////////////////////////////////////////////////////////////////////

namespace Math
{
  //! @brief Generate a random Gaussian number with 0.0 mean and 1.0 variance
  //!
  //! @param idum
  LIBRARY_API double gRand (long *idum);

  //! @brief Generate a random linear number between -1.0 and 1.0
  //!
  LIBRARY_API double lRand (long seed = -1);
}

/*
namespace Math
{
  //! Gaussian random number generator
  //!
  inline double ran1 (long *idum)
  {
    int j,k;
    static int iv[NTAB], iy = 0;
    void nrerror ();
    static double NDIV = 1.0 / (1.0 + (IM - 1.0) / NTAB);
    static double RNMX = (1.0 - EPS);
    static double AM = (1.0 / IM);

    if ((*idum <= 0) || (iy == 0))
    {
      *idum = max (-*idum, *idum);
      for (j = (NTAB + 7); j >= 0; --j) 
      {
        k = *idum / IQ;
        *idum = IA * (*idum - k * IQ) - IR * k;
        if (*idum < 0)
        {
          *idum += IM;
        }
        if (j < NTAB)
        {
          iv[j] = *idum;
        }
      }
      iy = iv[0];
    }
    k = *idum / IQ;
    *idum = IA * (*idum - k * IQ) - IR * k;
    if (*idum < 0)
    {
      *idum += IM;
    }

    j = iy * NDIV;
    iy = iv[j];
    iv[j] = *idum;
    return min (AM * iy, RNMX);
  }

  //! generate random # w/ 0 mean & 1.0 variance
  inline double gRand (long *idum)
  {
    static int iset = 0;
    static double gset;
    double fac, rsq, v1, v2;

    if (*idum < 0)
    {
      iset = 0;
    }
    if (iset == 0)
    {
      do
      {
        v1 = 2.0 * ran1(idum)-1.0;
        v2 = 2.0 * ran1(idum)-1.0;
        rsq = (v1 * v1) + (v2 * v2);
      } while (rsq >= 1.0 || rsq == 0.0);

      fac = sqrt(-2.0 * log(rsq) / rsq);
      gset = v1 * fac;
      iset = 1;
      return (v2 * fac);
    }
    else
    {
      iset = 0;
      return gset;
    }
  }
}
*/

#endif