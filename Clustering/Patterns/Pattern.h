///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Clustering
//  Workfile:        Pattern.h
//  Revision:        1.0 - 28 September, 2009
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Pattern type and collections container class definitions.  Assumes feature
//  vectors are of type double.  Base pattern collection requires file name
//  passed in to read values from disk.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef PATTERN_H
#define PATTERN_H

#include <vector>
#include "../../portable.h"

using namespace std;

namespace Clustering
{
  //! @ingroup Clustering
  //!
  //! @brief Pattern type description
  //!
  class LIBRARY_API Pattern
  {
  public:

    //! @brief Sizing constructor
    //!
    //! @param fdim Feature vector input size
    //! @param adim Attribute input size
    //!
    Pattern (int fdim, int adim);

    //! @brief Default destructor
    //!
    ~Pattern ();

    //! @brief Get the dimensions of a Pattern vector
    //!
    //! @return The size of a standard Pattern vector
    //!
    int getDimensions ();

    //! @brief Get the dimensions of a Pattern feature vector
    //!
    //! @return The size of a standard Pattern vector
    //!
    int getFeatureDimensions();

    //! @brief Get the dimensions of a Pattern vector
    //!
    //! @return The size of a standard Pattern vector
    //!
    int getAttributeDimensions();

    //! @brief Get a single raw feature element at a specified position
    //!
    //! @param i The element number of the feature vector being accessed
    //!
    //! @return The value of the feature element at i, or -1 if it doesn't
    //!         exist
    //!
    double getRawFeature (int i);

    //! @brief Grab the entire unscaled feature vector
    //!
    //! @param valVec Pointer to a data structure being populated by this
    //!               method
    //!
    void getRawFeatures (double *valVec);
    void getRawFeatures (vector<double> &valVec);

    //! @brief Access a single feature value
    //!
    //! @param i The index of the feature value being accessed
    //!
    //! @return The feature located at index i
    //!
    double getScaledFeature (int i);

    //! @brief Grab the entire scaled feature vector
    //!
    //! @param valVec Data structure being populated by this method
    //!
    void getScaledFeatures (double *valVec);
    void getScaledFeatures (vector<double> &valVec);

    //! @brief Access the attributes for this pattern
    //!
    //! @param valVec Data structure being populated by this method
    //!
    void getAttributes(double *valVec);
    void getAttributes(vector<double> &valVec);

    //! @brief Fills in the scaled feature vector provided the min and max
    //!        element values
    //!
    //! @param maxVals The vector of maximum element values
    //! @param minVals The vector of minimum element values
    //!
    void scaleFeatures (double *maxVals, double *minVals);
    void scaleFeatures (vector<double> &maxVals, vector<double> &minVals);

    //! @brief Modify the pattern attributes
    //!
    //! @param attribs The new values for the pattern attributes
    //!
    void setAttributes (double *attribs);
    void setAttributes (vector<double> &attribs);

    //! @brief Modify the feature vector with specified values
    //!
    //! @param valVec The new set of values to be set in the feature vector
    //!
    void setRawFeatures(double *valVec);
    void setRawFeatures(vector<double> &valVec);

  private:

    //! @brief Dimensions of the attribute vector
    //!
    int attribDims_;

    //! @brief Property of a training pattern used to infer cluster attribute
    //!
    double attribute_;

    //! @brief The attributes of the associated feature vector
    //!
    vector<double> attributes_;

    //! @brief The number of elements in the feature vector
    //!
    int dimensions_;

    //! @brief The raw input feature values used
    //!
    vector<double> rawFeatures_;

    //! @brief The input feature values scaled to be in the range [0, 1]
    //!
    vector<double> scaledFeatures_;
  }; // Pattern



  //! @ingroup Clustering
  //!
  //! @brief Patterns collection class description
  //!
  class LIBRARY_API Patterns
  {
  public:

    //! @brief Constructor that loads patterns from disk assuming 1D attributes
    //!
    //! @param fdims The dimensions of the feature vector
    //! @param adims The dimensions of the attribute vector
    //! @param fname The file name contaiing the pattern data
    //!
    Patterns(int fdims, int adims, char *fname);

    //! @brief Default destructor
    //!
    ~Patterns ();

    //! @brief Add an additional pattern to the list of patterns
    //!
    //! @param pat A new pattern to be added to the pattern collection
    //!
    //! @note Assumes that new feature vector is of appropriate length
    //!
    void addPattern (Pattern& pat);

    //! @brief Remove all patterns from the pattern collection
    //!
    void clearPatterns ();

    //! @brief Get the attribute of a specific pattern
    //!
    //! @param pat The pattern being accessed
    //!
    //! @return True if the specified pattern exists, false otherwise
    //!
    bool getPatternAttributes(int pat, double *valVec);
    bool getPatternAttributes(int pat, vector<double> &valVec);

    //! @brief Get the scale feature vector of a specific pattern
    //!
    //! @param pat    The pattern being accessed
    //! @param valVec The feature vector to be populated
    //!
    //! @note Assumes input vector has been allocated and is of appropriate length.
    //!       No error checking to verify pattern pat exists.
    //!
    void getPatternScaledFeatures (int pat, double *valVec);
    void getPatternScaledFeatures (int pat, vector<double> &valVec);

    //! @brief Get the raw feature vector of a specific pattern
    //!
    //! @param pat    The pattern being accessed
    //! @param valVec The feature vector to be populated
    //!
    //! @note Assumes input vector has been allocated and is of appropriate length.
    //!       No error checking to verify pattern pat exists.
    //!
    void getPatternRawFeatures(int pat, double *valVec);
    void getPatternRawFeatures(int pat, vector<double> &valVec);

    //! @brief Get the number of patterns defined
    //!
    int getPatternCount ();

    //! @brief Get the raw minimum and maximum values
    //!
    //! @param max_vals The maximum encountered values of all patterns
    //! @param min_vals The minimum encountered values of all patterns
    //!
    void getRanges (double *maxVals, double *minVals);

  private:

    //! @brief The array of patterns owned by this class
    //!
    vector<Pattern> patterns_;

    //! @brief The number of patterns associated with this object
    //!
    int nPatterns_;

    //! @brief The maximum values encountered for each feature in the
    //!        collection of feature vectors
    //!
    vector<double> maxFeatures_;

    //! @brief The minimum valeus encountered for each feature in the
    //!        collection of feature vectors
    //!
    vector<double> minFeatures_;

    //! @brief Read a pattern set from disk
    //!
    //! @param fdim  The number of elements in a feature vector
    //! @param adim  The number of elements in an attribute vector
    //! @param fname The file name containing the pattern data
    //!
    void readPatternsFromDisk (int fdim, int adim, char *fname);

    //! @brief Fill in the maxFeatures_ and minFeatures_ vectors based on the
    //!        input data set
    //!
    void findMinMax ();

    //! @brief Scale the feature values in all patterns
    //!
    void scaleFeatures ();
  }; // Pattern
} // Clustering

#endif