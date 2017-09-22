///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Math
//  Workfile:        VectorMath.cpp
//  Revision:        1.0 - 24 November, 2008
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Collection of math functions applicable to numerical vectors
//
///////////////////////////////////////////////////////////////////////////////

#include "VectorMath.h"

namespace Math
{
  void merge (double *in1, double *in2,
              int *indx1, int *indx2,
              int size1, int size2)

  {
    double dTemp;
    double *inTemp1, *inTemp2;
    int *indxTemp1, *indxTemp2, sizeTemp1, sizeTemp2;
    int iTemp, id1, id2, x;

    if (size1 == 1)
    {
      //! Cease breaking down & return values
      if (size2 > 1)
      {
        //! In the event that indx1 == 1 && indx2 == 2
        if (in2[0] > in2[1])
        {
          dTemp = in2[0];
          iTemp = indx2[0];
          in2[0] = in2[1];
          indx2[0] = indx2[1];
          in2[1] = dTemp;
          indx2[1] = iTemp;
        }
      }
    }
    else
    {
      //! Break the two vectors in half and sort
      //! Vector1
      sizeTemp1 = size1 / 2;
      sizeTemp2 = size1 - sizeTemp1;

      inTemp1 = new double[sizeTemp1];
      indxTemp1 = new int[sizeTemp1];
      inTemp2 = new double[sizeTemp2];
      indxTemp2 = new int[sizeTemp2];

      for (x = 0; x < sizeTemp1; ++x)
      {
        inTemp1[x] = in1[x];
        indxTemp1[x] = indx1[x];
      }
      for (x = 0; x < sizeTemp2; ++x)
      {
        inTemp2[x] = in1[x + sizeTemp1];
        indxTemp2[x] = indx1[x + sizeTemp1];
      }

      merge (inTemp1, inTemp2, indxTemp1, indxTemp2, sizeTemp1, sizeTemp2);

      id1 = id2 = 0;
      for (x = 0; x < size1; ++x)
      {
        if (id1 >= sizeTemp1)
        {
          in1[x] = inTemp2[id2];
          indx1[x] = indxTemp2[id2];
          ++id2;
        }
        else if (id2 >= sizeTemp2)
        {
          in1[x] = inTemp1[id1];
          indx1[x] = indxTemp1[id1];
          ++id1;
        }
        else if (inTemp1[id1] < inTemp2[id2])
        {
          in1[x] = inTemp1[id1];
          indx1[x] = indxTemp1[id1];
          ++id1;
        }
        else
        {
          in1[x] = inTemp2[id2];
          indx1[x] = indxTemp2[id2];
          ++id2;
        }
      }

      delete [] inTemp1;
      delete [] inTemp2;
      delete [] indxTemp1;
      delete [] indxTemp2;


      //! Vector2
      sizeTemp1 = size2 / 2;
      sizeTemp2 = size2 - sizeTemp1;

      inTemp1 = new double[sizeTemp1];
      indxTemp1 = new int[sizeTemp1];
      inTemp2 = new double[sizeTemp2];
      indxTemp2 = new int[sizeTemp2];

      for (x = 0; x < sizeTemp1; ++x)
      {
        inTemp1[x] = in2[x];
        indxTemp1[x] = indx2[x];
      }
      for (x = 0; x < sizeTemp2; ++x)
      {
        inTemp2[x] = in2[x + sizeTemp1];
        indxTemp2[x] = indx2[x + sizeTemp1];
      }

      merge (inTemp1, inTemp2, indxTemp1, indxTemp2, sizeTemp1, sizeTemp2);

      id1 = id2 = 0;
      for (x = 0; x < size2; ++x)
      {
        if (id1 >= sizeTemp1)
        {
          in2[x] = inTemp2[id2];
          indx2[x] = indxTemp2[id2];
          ++id2;
        }
        else if (id2 >= sizeTemp2)
        {
          in2[x] = inTemp1[id1];
          indx2[x] = indxTemp1[id1];
          ++id1;
        }
        else if (inTemp1[id1] < inTemp2[id2])
        {
          in2[x] = inTemp1[id1];
          indx2[x] = indxTemp1[id1];
          ++id1;
        }
        else
        {
          in2[x] = inTemp2[id2];
          indx2[x] = indxTemp2[id2];
          ++id2;
        }
      }

      delete [] inTemp1;
      delete [] inTemp2;
      delete [] indxTemp1;
      delete [] indxTemp2;
    } // if (size1 == 1) ... else
  }


  LIBRARY_API void mergeSort (vector<double> &vals, vector<int> &indexes)
  {
    double *in1, *in2;
    int *indx1, *indx2;
    unsigned int half1, half2;
    unsigned int id1, id2;
    unsigned int x;

    if (vals.size() <= 1)
    {
      for (x = 0; x < vals.size(); ++x)
      {
        indexes.at (x) = x;
      }
      return;
    }

    half1 = vals.size () / 2;
    half2 = vals.size () - half1;
    in1 = new double[half1];
    indx1 = new int[half1];
    in2 = new double[half2];
    indx2 = new int[half2];

    for (x = 0; x < half1; ++x)
    {
      in1[x] = vals.at(x);
      indx1[x] = x;
    }
    for (x = 0; x < half2; ++x)
    {
      in2[x] = vals.at(half1+x);
      indx2[x] = half1+x;
    }

//    vals.clear();
//    indexes.clear();

    merge (in1, in2, indx1, indx2, half1, half2);

    //! bring the two sorted halfs together
    id1 = id2 = x = 0;
    while (id1 < half1 || id2 < half2)
    {
      if (id1 >= half1)
      {
        vals.at (x) = in2[id2];
        indexes.at (x) = indx2[id2];
//        vals.push_back (in2[id2]);
//        indexes.push_back (indx2[id2]);
        ++id2;
      }
      else if (id2 >= half2)
      {
        vals.at (x) = in1[id1];
        indexes.at (x) = indx1[id1];
//        vals.push_back (in1[id1]);
//        indexes.push_back (indx1[id1]);
        ++id1;
      }
      else if (in1[id1] < in2[id2])
      {
        vals.at (x) = in1[id1];
        indexes.at (x) = indx1[id1];
//        vals.push_back (in1[id1]);
//        indexes.push_back (indx1[id1]);
        ++id1;
      }
      else
      {
        vals.at (x) = in2[id2];
        indexes.at (x) = indx2[id2];
//        vals.push_back (in2[id2]);
//        indexes.push_back (indx2[id2]);
        ++id2;
      }
      ++x;
    } // while (id1 < half1 && id2 < half2)
  }


  LIBRARY_API double eucDist (vector<double> &val1, vector<double> &val2)
  {
    double temp = 0.0f;
    vector<double>::iterator iter1, iter2;
    int s1 = val1.size(), s2 = val2.size();
    if (val1.size() != val2.size() || val1.size() < 1 || val2.size() < 1)
    {
      return -1.0;
    }

    iter1 = val1.begin();
    iter2 = val2.begin();
    for (; iter1 != val1.end(); ++iter1, ++iter2)
    {
      temp += ((*iter1 - *iter2) * (*iter1 - *iter2));
    }
    return sqrt(temp);
  }

  LIBRARY_API int maxElement (double *val1, int dim)
  {
    double max;
    int k, kbest = 0;
    if (val1 == NULL)
    {
      return -1;
    }

    max = val1[0];
    kbest = 0;

    for (k = 1; k < dim; k++)
    {
      if (val1[k] > max)
      {
        max = val1[k];
        kbest = k;
      }
    }
    return kbest;
  }


  /*
    double dotProduct (numVector &val1, numVector &val2)
  {
    if (val1.vals != val2.vals)
    {
      return 0.0f;
    }
    int y; 
    double out = 0.0f;

    for (y = 0; y < val1.vals; ++y)
    {
      out += (val1.data[y] * val2.data[y]);
    }
    return out;
  }

  bool crossProduct (numVector &val1, numVector &val2, numVector &out)
  {
    //! Currently only supports 3D vectors
    if (val1.vals != 3 || val2.vals != 3)
    {
      return false;
    }
    out.data[0] = (val1.data[1] * val2.data[2]) - (val1.data[2] * val2.data[1]);
    out.data[1] = (val1.data[2] * val2.data[0]) - (val1.data[0] * val2.data[2]);
    out.data[2] = (val1.data[0] * val2.data[1]) - (val1.data[1] * val2.data[0]);
    return true;
  }
  */
}
