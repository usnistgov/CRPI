///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Clustering
//  Workfile:        NumPattern.h
//  Revision:        1.0 - 28 September, 2009
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Pattern feater definitions
//
///////////////////////////////////////////////////////////////////////////////

#ifndef NUMPATTERN_H
#define NUMPATTERN_H

namespace Clustering
{
  class LIBRARY_API DoubleFeature
  {
  public:

    //! @brief Default constructor
    //!
    DoubleFeature ();

    //! @brief Default destructor
    //!
    ~DoubleFeature ();

    //! @brief TODO
    //!
    DoubleFeature bad();

    //! @brief TODO
    //!
    DoubleFeature empty();

    //! @brief Cast class data to double-precision floating point number
    //!
    double toDouble ();

    //! @brief Feature assignment function
    //!
    //! @param source An existing feature value that will be used to populate
    //!               this feature instance
    //!
    DoubleFeature & operator=(const DoubleFeature &source);

    DoubleFeature & operator=(const double &source);

    const DoubleFeature operator+(const DoubleFeature &other) const;

    const DoubleFeature operator+(const double &other) const;

    const DoubleFeature operator-(const DoubleFeature &other) const;

    const DoubleFeature operator-(const double &other) const;

    const DoubleFeature operator*(const DoubleFeature &other) const;

    const DoubleFeature operator*(const double &other) const;

    const DoubleFeature operator/(const DoubleFeature &other) const;

    const DoubleFeature operator/(const double &other) const;

    bool operator==(const DoubleFeature &other) const;

    bool operator!=(const DoubleFeature &other) const;

    bool operator>(const DoubleFeature &other) const;

    bool operator>=(const DoubleFeature &other) const;

    bool operator<(const DoubleFeature &other) const;

    bool operator<=(const DoubleFeature &other) const;

  private:

    //! @brief Numerical value held by the feature class
    //!
    double num;
  }; // DoubleFeature

} // Clustering

#endif