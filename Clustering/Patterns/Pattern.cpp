///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Clustering
//  Workfile:        Pattern.cpp
//  Revision:        1.0 - 28 September, 2009
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Pattern type and collections container class declarations.
//
///////////////////////////////////////////////////////////////////////////////

#include "Pattern.h"
#include <cmath>
#include <fstream>

namespace Clustering
{
  /////////////////////////////////////////////////////////////////////////////
  //                          PATTERN DEFINITIONS                            //
  /////////////////////////////////////////////////////////////////////////////

  LIBRARY_API Pattern::Pattern(int fdim, int adim) :
    attribute_(0.0f),
    dimensions_(fdim),
    attribDims_(adim)
  {
  }


  LIBRARY_API Pattern::~Pattern()
  {
    if (dimensions_ > 0)
    {
      rawFeatures_.clear();
      scaledFeatures_.clear();
    }
  }


  LIBRARY_API int Pattern::getDimensions()
  {
    return dimensions_;
  }


  LIBRARY_API int Pattern::getFeatureDimensions()
  {
    return dimensions_;
  }


  LIBRARY_API int Pattern::getAttributeDimensions()
  {
    return attribDims_;
  }


  LIBRARY_API double Pattern::getRawFeature (int i)
  {
    if (dimensions_ > i)
    {
      return rawFeatures_.at(i);
    }
    else
    {
      return -1.0f;
    }
  }


  LIBRARY_API void Pattern::getRawFeatures (double *valVec)
  {
    int i;
    for (i = 0; i < dimensions_; ++i)
    {
      valVec[i] = rawFeatures_.at(i);  
    }
  }


  LIBRARY_API void Pattern::getRawFeatures(vector<double> &valVec)
  {
    valVec.clear();
    for (int i = 0; i < dimensions_; ++i)
    {
      valVec.push_back(rawFeatures_.at(i));
    }
  }


  LIBRARY_API double Pattern::getScaledFeature (int i)
  {
    if (dimensions_ > i)
    {
      return scaledFeatures_.at(i);
    }
    else
    {
      return -1.0f;
    }
  }


  LIBRARY_API void Pattern::getScaledFeatures (double *valVec)
  {
    int i;

    for (i = 0; i < dimensions_; ++i)
    {
      valVec[i] = scaledFeatures_.at(i);  
    }
  }


  LIBRARY_API void Pattern::getScaledFeatures(vector<double> &valVec)
  {
    valVec.clear();
    int i;
    for (i = 0; i < dimensions_; ++i)
    {
      valVec.push_back(scaledFeatures_.at(i));
    }
  }


  LIBRARY_API void Pattern::getAttributes(double *valVec)
  {
    int i;

    for (i = 0; i < attribDims_; ++i)
    {
      valVec[i] = attributes_.at(i);
    }
  }


  LIBRARY_API void Pattern::getAttributes(vector<double> &valVec)
  {
    valVec.clear();
    for (int i = 0; i < attribDims_; ++i)
    {
      valVec.push_back(attributes_.at(i));
    }
  }


  LIBRARY_API void Pattern::scaleFeatures(double *maxVals, double *minVals)
  {
    int ifeat;
    double val;

    scaledFeatures_.clear();
    for (ifeat = 0; ifeat < dimensions_; ++ifeat)
    {
      if (fabs(maxVals[ifeat] - minVals[ifeat]) < 0.00001f)
      {
        val = 1.0f;
      }
      else
      {
        val = (rawFeatures_[ifeat] - minVals[ifeat]) /
              (maxVals[ifeat] - minVals[ifeat]);
      }
      scaledFeatures_.push_back(val);
    }
  }


  LIBRARY_API void Pattern::scaleFeatures (vector<double> &maxVals, vector<double> &minVals)
  {
    int ifeat;
    double val;

    scaledFeatures_.clear();
    for (ifeat = 0; ifeat < dimensions_; ++ifeat)
    {
      if (fabs(maxVals.at(ifeat) - minVals.at(ifeat)) < 0.00001f)
      {
        val = 1.0f;
      }
      else
      {
        val = (rawFeatures_.at(ifeat) - minVals.at(ifeat)) /
              (maxVals.at(ifeat) - minVals.at(ifeat));
      }
      scaledFeatures_.push_back(val);
    }
  }


  LIBRARY_API void Pattern::setAttributes(double *valVec)
  {
    int i;
    attributes_.clear();
    for (i = 0; i < attribDims_; ++i)
    {
      attributes_.push_back(valVec[i]);
    }
  }


  LIBRARY_API void Pattern::setAttributes(vector<double> &valVec)
  {
    int i;
    attributes_.clear();
    for (i = 0; i < attribDims_; ++i)
    {
      attributes_.push_back(valVec.at(i));
    }
  }


  LIBRARY_API void Pattern::setRawFeatures (double *valVec)
  {
    int i;
    rawFeatures_.clear();
    for (i = 0; i < dimensions_; ++i)
    {
      rawFeatures_.push_back (valVec[i]);
    }
  }

  LIBRARY_API void Pattern::setRawFeatures(vector<double> &valVec)
  {
    int i;
    rawFeatures_.clear();
    for (i = 0; i < dimensions_; ++i)
    {
      rawFeatures_.push_back(valVec.at(i));
    }
  }






  /////////////////////////////////////////////////////////////////////////////
  //                    PATTERN COLLECTION DEFINITIONS                       //
  /////////////////////////////////////////////////////////////////////////////

  LIBRARY_API Patterns::Patterns (int fdims, int adims, char *fname) :
    nPatterns_(0)
  {
    minFeatures_.resize(fdims);
    maxFeatures_.resize(fdims);

    if (fname != NULL)
    {
      //! Get the pattern data from disk
      readPatternsFromDisk (fdims, 1, fname);
      //! Find the min and max feature values for each feature element
      findMinMax();
      //! Fill in the scaled feature vector using the above min and max values
      scaleFeatures();
    }
  }


  LIBRARY_API Patterns::~Patterns ()
  {
    patterns_.clear();
    minFeatures_.clear();
    maxFeatures_.clear();
  }


  LIBRARY_API void Patterns::addPattern (Pattern& pat)
  {
    double *feat = new double[pat.getDimensions()];
    double *attribs = new double[pat.getAttributeDimensions()];

    pat.getRawFeatures (feat);
    pat.getAttributes (attribs);
    Pattern newPat (pat.getDimensions(), pat.getAttributeDimensions());
    newPat.setRawFeatures (feat);
    newPat.setAttributes (attribs);

    patterns_.push_back (newPat);
    nPatterns_ = patterns_.size();
    findMinMax ();
    scaleFeatures ();
  }


  LIBRARY_API void Patterns::clearPatterns ()
  {
    patterns_.clear();
    nPatterns_ = patterns_.size();
  }


  LIBRARY_API bool Patterns::getPatternAttributes(int pat, double *valVec)
  {
    if (pat < nPatterns_ && pat >= 0)
    {
      vector<double> vals;
      patterns_.at(pat).getAttributes(vals);
      double dim = patterns_.at(pat).getAttributeDimensions();

      for (int i = 0; i < dim; ++i)
      {
        valVec[i] = vals.at(i);
      }
      return true;
    }
    return false;
  }


  LIBRARY_API bool Patterns::getPatternAttributes(int pat, vector<double> &valVec)
  {
    if (pat < nPatterns_ && pat >= 0)
    {
      vector<double> vals;
      patterns_.at(pat).getAttributes(vals);
      double dim = patterns_.at(pat).getAttributeDimensions();
      valVec.clear();
      for (int i = 0; i < dim; ++i)
      {
        valVec.push_back(vals.at(i));
      }
      return true;
    }
    return false;
  }


  LIBRARY_API void Patterns::getPatternScaledFeatures (int pat, double *valVec)
  {
    if (pat < nPatterns_)
    {
      patterns_.at(pat).getScaledFeatures (valVec);
    }
  }


  LIBRARY_API void Patterns::getPatternScaledFeatures(int pat, vector<double> &valVec)
  {
    if (pat < nPatterns_ && pat >= 0)
    {
      patterns_.at(pat).getScaledFeatures (valVec);
    }
  }


  LIBRARY_API void Patterns::getPatternRawFeatures(int pat, double *valVec)
  {
    if (pat < nPatterns_)
    {
      patterns_.at(pat).getRawFeatures(valVec);
    }
  }


  LIBRARY_API void Patterns::getPatternRawFeatures(int pat, vector<double> &valVec)
  {
    if (pat < nPatterns_ && pat >= 0)
    {
      patterns_.at(pat).getRawFeatures(valVec);
    }
  }


  LIBRARY_API int Patterns::getPatternCount ()
  {
    return nPatterns_;
  }


  LIBRARY_API void Patterns::readPatternsFromDisk (int fdim, int adim, char *fname)
  {
    vector<double> feats, attribs;
    double val;
    int i;
    Pattern *newpat;


    ifstream newfile (fname);
    if (!newfile)
    {
      //! File does not exist;
      return;
    }

    while (newfile >> val)
    {
      feats.push_back(val);

      //! Read the feature values
      for (i = 1; i < fdim; ++i)
      {
        newfile >> val;
        feats.push_back(val);
      }

      for (i = 0; i < adim; ++i)
      {
        newfile >> val;
        attribs.push_back(val);
      }

      newpat = new Pattern(fdim, adim);

      //! Set feature values and attribute into a new pattern object
      newpat->setRawFeatures (feats);
      newpat->setAttributes (attribs);

      patterns_.push_back(*newpat);
      ++nPatterns_;
      newpat = NULL;
    }
  }


  LIBRARY_API void Patterns::findMinMax ()
  {
    int ifeat, ipat;
    if (nPatterns_ < 1)
    {
      return;
    }

    //! Find minimum and maximum values for each feature element over all patterns
    for (ifeat = 0; ifeat < patterns_.at(0).getDimensions(); ++ifeat)
    {
      //! initialize min and max with first pattern
      maxFeatures_.at(ifeat) = patterns_.at(0).getRawFeature (ifeat);
      minFeatures_.at(ifeat) = patterns_.at(0).getRawFeature (ifeat);

      for (ipat = 1; ipat < nPatterns_; ++ipat)
      {
        if (patterns_.at(ipat).getRawFeature (ifeat) > maxFeatures_.at(ifeat))
        {
          maxFeatures_.at(ifeat) = patterns_.at(ipat).getRawFeature (ifeat);
        }

        if (patterns_.at(ipat).getRawFeature (ifeat) < minFeatures_.at(ifeat))
        {
          minFeatures_.at(ifeat) = patterns_.at(ipat).getRawFeature (ifeat);
        }
      } // for (ipat = 1; ipat < nPatterns_; ++ipat)
    } // for (ifeat = 0; ifeat < patterns_.getDimensions(); ++ifeat)
  }


  LIBRARY_API void Patterns::scaleFeatures ()
  {
    int ipat;

    //! Requires that maximum and minimum values for all elements over all
    //! patterns have been defined.  Using these values, the raw feature values
    //! are scaled.
    for (ipat = 0; ipat < nPatterns_; ++ipat)
    {
      patterns_.at(ipat).scaleFeatures (maxFeatures_, minFeatures_);
    }
  }


  LIBRARY_API void Patterns::getRanges (double *maxVals, double *minVals)
  {
    int i;
    for (i = 0; i < patterns_[0].getDimensions(); ++i)
    {
      maxVals[i] = maxFeatures_.at(i);
      minVals[i] = minFeatures_.at(i);
    }
  }
} // Clustering