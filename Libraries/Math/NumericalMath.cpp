///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Math
//  Workfile:        NumericalMath.cpp
//  Revision:        1.0 - 21 January, 2008
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Collection of numerical math functions.
//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//   From "Numerical Recipes in C" for normal-distribution random numbers.   //
//          I did not write these, and cannot take credit for them.          //
///////////////////////////////////////////////////////////////////////////////

#include "NumericalMath.h"

namespace Math
{
  //! Gaussian random number generator
  //!
  double ran1 (long *idum)
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

    j = (int)(iy * NDIV);
    iy = iv[j];
    iv[j] = *idum;
    return min (AM * iy, RNMX);
  }

  //! generate random # w/ 0 mean & 1.0 variance
  LIBRARY_API double gRand (long *idum)
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


  LIBRARY_API double lRand (long seed)
  {
    double val;
    if (seed > -1)
    {
      srand (seed);
    }

    val = 10000.0f - (double)(rand() % 20000);
    val /= 10000.0f;

    return val;
  }

}
