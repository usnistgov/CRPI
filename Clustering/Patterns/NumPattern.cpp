///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Neural Tissue
//  Subsystem:       Clustering
//  Workfile:        NumPattern.cpp
//  Revision:        1.0 - 28 September, 2009
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Pattern feater definitions
//
///////////////////////////////////////////////////////////////////////////////

#include "NumPattern.h"

namespace Clustering
{
  LIBRARY_API DoubleFeature::DoubleFeature ()
  {
    num = 0.0f;
  }

  LIBRARY_API DoubleFeature::~DoubleFeature ()
  {
  }

  LIBRARY_API DoubleFeature DoubleFeature::bad()
  {
    DoubleFeature temp;
    temp.num = -1.0f;
    return temp;
  }

  LIBRARY_API DoubleFeature DoubleFeature::empty()
  {
    DoubleFeature temp;
    temp.num = 0.0f;
    return temp;
  }

  LIBRARY_API double DoubleFeature::toDouble()
  {
    return num;
  }

  LIBRARY_API DoubleFeature & DoubleFeature::operator=(const DoubleFeature &source)
  {
    if (this != &source)
    {
      num = source.num;
    }
    return *this;
  }

  LIBRARY_API DoubleFeature & DoubleFeature::operator=(const double &source)
  {
    this->num = source;
    return *this;
  }


  LIBRARY_API const DoubleFeature DoubleFeature::operator+(const DoubleFeature &other) const
  {
    DoubleFeature result = *this;
    result.num += other.num;
    return result;
  }

  LIBRARY_API const DoubleFeature DoubleFeature::operator+(const double &other) const
  {
    DoubleFeature result = *this;
    result.num += other;
    return result;
  }

  LIBRARY_API const DoubleFeature DoubleFeature::operator-(const DoubleFeature &other) const
  {
    DoubleFeature result = *this;
    result.num -= other.num;
    return result;
  }

  LIBRARY_API const DoubleFeature DoubleFeature::operator-(const double &other) const
  {
    DoubleFeature result = *this;
    result.num -= other;
    return result;
  }

  LIBRARY_API const DoubleFeature DoubleFeature::operator*(const DoubleFeature &other) const
  {
    DoubleFeature result = *this;
    result.num *= other.num;
    return result;
  }

  LIBRARY_API const DoubleFeature DoubleFeature::operator*(const double &other) const
  {
    DoubleFeature result = *this;
    result.num *= other;
    return result;
  }

  LIBRARY_API const DoubleFeature DoubleFeature::operator/(const DoubleFeature &other) const
  {
    DoubleFeature result = *this;
    result.num /= other.num;
    return result;
  }

  LIBRARY_API const DoubleFeature DoubleFeature::operator/(const double &other) const
  {
    DoubleFeature result = *this;
    result.num /= other;
    return result;
  }

  LIBRARY_API bool DoubleFeature::operator==(const DoubleFeature &other) const
  {
    return (this->num == other.num);
  }

  LIBRARY_API bool DoubleFeature::operator!=(const DoubleFeature &other) const
  {
    return (this->num != other.num);
  }

  LIBRARY_API bool DoubleFeature::operator>(const DoubleFeature &other) const
  {
    return (this->num > other.num);
  }

  LIBRARY_API bool DoubleFeature::operator>=(const DoubleFeature &other) const
  {
    return (this->num >= other.num);
  }

  LIBRARY_API bool DoubleFeature::operator<(const DoubleFeature &other) const
  {
    return (this->num < other.num);
  }

  LIBRARY_API bool DoubleFeature::operator<=(const DoubleFeature &other) const
  {
    return (this->num <= other.num);
  }

} // Clustering
