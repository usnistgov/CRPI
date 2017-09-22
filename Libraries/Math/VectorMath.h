///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Math
//  Workfile:        VectorMath.h
//  Revision:        1.0 - 24 November, 2008
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Collection of vector-based mathematical and data-manipulative functions
//
///////////////////////////////////////////////////////////////////////////////

#ifndef VECTOR_MATH_H
#define VECTOR_MATH_H

#include "../../portable.h"
#include <vector>
#if defined(_MSC_VER)
#include "math.h"
#elif defined(__GNUC__)
#include <cmath>
#include <stdio.h>
#endif

using namespace std;

///////////////////////////////////////////////////////////////////////////////

namespace Math
{
  //! @brief Cartesian point structure
  //!
  struct point
  {
    //! @brief Cartesian X axis coordinate
    //!
    double x;

    //! @brief Cartesian Y axis coordinate
    //!
    double y;

    //! @brief Cartesian Z axis coordinate
    //!
    double z;

    //! @brief Default constructor
    //!
    point()
    {
      x = y = z = 0.0;
    }

    //! @brief Copy constructor
    //!
    point(const point& source)
    {
      x = source.x;
      y = source.y;
      z = source.z;
    }

    //! @brief Assignmet constructor
    //!
    point(double px, double py, double pz) :
      x(px),
      y(py),
      z(pz)
    {
    }

    //! @brief Point assignment function
    //!
    //! @param source An existing point that will be used to populate this point
    //!               instance
    //!
    point & operator=(const point &source)
    {
      if (this != &source)
      {
        x = source.x;
        y = source.y;
        z = source.z;
      }
      return *this;
    }

    point operator/(double &val)
    {
      point pC;
      pC.x = x / val;
      pC.y = y / val;
      pC.z = z / val;
      return pC;
    }

    //! @brief Point vector summation
    //!
    //! @param pB The source additive point vector
    //!
    //! @return The sum of the two point vectors
    //!
    point operator+(const point &pB)
    {
      point pC;
      pC.x = x + pB.x;
      pC.y = y + pB.y;
      pC.z = z + pB.z;
      return pC;
    }

    //! @brief Point vector difference
    //!
    //! @param pB The source subtractive point vector
    //!
    //! @return The difference of the two point vectors
    //!
    point operator-(const point &pB)
    {
      point pC;
      pC.x = x - pB.x;
      pC.y = y - pB.y;
      pC.z = z - pB.z;
      return pC;
    }

    //! @brief Point vector magnitude
    //!
    //! @return The magnitude of the point vector
    //!
    double magnitude()
    {
      return sqrt((x*x) + (y*y) + (z*z));
    }

    //! @brief Compute the Cartesian distance to another point
    //!
    //! @param dest The comparitive point to which we are finding the distance
    //! @param root Whether or not to take the square root (false makes the
    //!             equation faster--useful when only doing nearest-neighbor
    //!             comparisons)
    //!
    //! @return The Cartesian distance between this point and a destination
    //!         point
    //!
    double distance(const point &dest, bool root = false)
    {
      static double temp;
      temp = (dest.x - x) * (dest.x - x);
      temp += (dest.y - y) * (dest.y - y);
      temp += (dest.z - z) * (dest.z - z);
      return (root ? sqrt(temp) : temp);
    }

    //! @brief Compute the cross product with another point
    //!
    point cross(const point& val)
    {
      point pC;
      pC.x = (y * val.z) - (z * val.y);
      pC.y = (z * val.x) - (x * val.z);
      pC.z = (x * val.y) - (y * val.x);
      return pC;
    }

    //! @brief Compute the dot product with another point
    //!
    double dot(const point& val)
    {
      return ((x * val.x) + (y * val.y) + (z * val.z));
    }

    //! @brief Print out the point to the screen
    //!
    void print()
    {
      printf("(%f, %f, %f)\n", x, y, z);
    }
  };


  //! @brief Cartesian pose structure
  //!
  struct pose
  {
    //! @brief Cartesian X axis coordinate
    //!
    double x;

    //! @brief Cartesian Y axis coordinate
    //!
    double y;

    //! @brief Cartesian Z axis coordinate
    //!
    double z;

    //! @brief Cartesian X axis rotation
    //!
    double xr;

    //! @brief Cartesian Y axis rotation
    //!
    double yr;

    //! @brief Cartesian Z axis rotation
    //!
    double zr;

    //! @brief Default constructor
    //!
    pose()
    {
      x = y = z = xr = yr = zr = 0.0f;
    }

    //! @brief Copy constructor
    //!
    pose(const pose& source)
    {
      x = source.x;
      y = source.y;
      z = source.z;
      xr = source.xr;
      yr = source.yr;
      zr = source.zr;
    }

    //! @brief Assignmet constructor
    //!
    pose(double px, double py, double pz, double pxr, double pyr, double pzr) :
      x(px),
      y(py),
      z(pz),
      xr(pxr),
      yr(pyr),
      zr(pzr)
    {
    }

    //! @brief Point assignment function
    //!
    //! @param source An existing point that will be used to populate this point
    //!               instance
    //!
    pose & operator=(const pose &source)
    {
      if (this != &source)
      {
        x = source.x;
        y = source.y;
        z = source.z;
        xr = source.xr;
        yr = source.yr;
        zr = source.zr;
      }
      return *this;
    }

    pose operator/(double &val)
    {
      pose pC;
      pC.x = x / val;
      pC.y = y / val;
      pC.z = z / val;
      pC.xr = xr / val;
      pC.yr = yr / val;
      pC.zr = zr / val;
      return pC;
    }

    pose operator*(double &val)
    {
      pose pC;
      pC.x = x * val;
      pC.y = y * val;
      pC.z = z * val;
      pC.xr = xr * val;
      pC.yr = yr * val;
      pC.zr = zr * val;
      return pC;
    }

    //! @brief Point vector summation
    //!
    //! @param pB The source additive point vector
    //!
    //! @return The sum of the two point vectors
    //!
    pose operator+(const pose &pB)
    {
      pose pC;
      pC.x = x + pB.x;
      pC.y = y + pB.y;
      pC.z = z + pB.z;
      pC.xr = xr + pB.xr;
      pC.yr = yr + pB.yr;
      pC.zr = zr + pB.zr;
      return pC;
    }

    //! @brief Point vector difference
    //!
    //! @param pB The source subtractive point vector
    //!
    //! @return The difference of the two point vectors
    //!
    pose operator-(const pose &pB)
    {
      pose pC;
      pC.x = x - pB.x;
      pC.y = y - pB.y;
      pC.z = z - pB.z;
      pC.xr = xr - pB.xr;
      pC.yr = yr - pB.yr;
      pC.zr = zr - pB.zr;
      return pC;
    }

    //! @brief Print out the point to the screen
    //!
    void print()
    {
      printf("(%f, %f, %f, %f, %f, %f)\n", x, y, z, xr, yr, zr);
    }
  };


  //! @brief Perform an in-place merge-sort on a vector of floating point
  //!        numbers
  //!
  //! @param vals    The vector of values to be sorted
  //! @param indexes The vector of index values corresponding to the original
  //!                order of the elements in the input vector
  //!
  LIBRARY_API void mergeSort (vector<double> &vals, vector<int> &indexes);

  //! @brief Compute the Euclidean distance between two point vectors
  //!
  //! @param val1 The first vector of numbers to be distanced (origin)
  //! @param val2 The second vector of numbers to be distanced (target)
  //!
  //! @return The unit distance between the two point vectors, or -1 if the
  //!         two vectors are not equally lengthed or are of illegal length
  //!
  LIBRARY_API double eucDist (vector<double> &val1, vector<double> &val2);

  //! @brief Return the index of the maximum value in a vector
  //!
  //! @param val1 The vector being queried
  //! @param dim  The dimensions of the vector being queried
  //!
  //! @return The index of the largest element in the vector, -1 if the vector
  //!         is empty
  //!
  LIBRARY_API int maxElement (double *val1, int dim);
}

#endif