///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Clustering
//  Workfile:        kMeansCluster.cpp
//  Revision:        1.0 - 28 September, 2009
//  Author:          J. Marvel
//
//  Description
//  ===========
//  K-Means clustering interface definition source file.
//
///////////////////////////////////////////////////////////////////////////////

#include "kMeansCluster.h"
#include <time.h>

namespace Clustering
{
  LIBRARY_API kMeans::kMeans (int fdim, int adim, int kclust, char *path) :
                              numClusters_(kclust),
                              featureDims_(fdim),
                              attributeDims_(adim),
                              memClusterPattern_(NULL),
                              patternAssignments_(NULL),
                              minClusterMembers_(0)
  {
    int i, j, k;
    clusters_ = new Clusters(kclust, fdim, adim);
    clusters_->setMinMembers(minClusterMembers_);

    trainingPatterns_ = new Patterns (fdim, adim, path);
    numPatterns_ = trainingPatterns_->getPatternCount ();

    //! Allocate space for the membership association matrix
    memClusterPattern_ = new bool *[numClusters_];
    for (i = 0; i < numClusters_; ++i)
    {
      memClusterPattern_[i] = NULL;
    }

    if (numPatterns_ > 0)
    {
      //! Create vector for pattern assignments to clusters
      patternAssignments_ = new int[numPatterns_];

      for (i = 0; i < numPatterns_; ++i)
      {
        //! Set to "unassigned"
        patternAssignments_[i] = -1;
      }

      //! Allocate space for the membership association matrix
      for (i = 0; i < numClusters_; ++i)
      {
        memClusterPattern_[i] = new bool[numPatterns_];
      }

      //! Assign all patterns to un-associated
      for (j = 0; j < numClusters_; ++j)
      {
        for (k = 0; k < numPatterns_; ++k)
        {
          memClusterPattern_[j][k] = false;
        } // for (k = 0; k < numPatterns_; ++k)
      } // for (j = 0; j < numClusters_; ++j)
    } // if (numPatterns_ > 0)
  }


  LIBRARY_API kMeans::~kMeans()
  {
    int i;
    for (i = 0; i < numClusters_; ++i)
    {
      delete [] memClusterPattern_ [i];
    }
    delete [] memClusterPattern_;
    delete [] patternAssignments_;
  }


  LIBRARY_API void kMeans::setMinClusterMembers(int min)
  {
    minClusterMembers_ = min;
    clusters_->setMinMembers(min);
  }


  LIBRARY_API void kMeans::seedClusters ()
  {
    int p, k = 0, km;
    vector<double> valVec;
    valVec.resize(featureDims_);

    vector<double> attribs;
    attribs.resize(attributeDims_);

    for (p = 0; p < numClusters_; ++p)
    {
      patternAssignments_[p] = p;

      memClusterPattern_[p][p] = true;

      trainingPatterns_->getPatternRawFeatures(p, valVec);
      trainingPatterns_->getPatternAttributes(p, attribs);

      //! Install affiliation in Clusters array: 
      clusters_->addMember(p, valVec, attribs);
    }

    srand((unsigned)time(NULL));
    for (; p < numPatterns_; ++p)
    {
      k = (rand() * numClusters_ / (RAND_MAX + 1));
      patternAssignments_[p] = k;

      memClusterPattern_[k][p] = true;

      trainingPatterns_->getPatternRawFeatures(p, valVec);
      trainingPatterns_->getPatternAttributes(p, attribs);

      //! Install affiliation in Clusters array: 
      clusters_->addMember(k, valVec, attribs);
    }





    /*
    //! Seed the clusters by picking random patterns and assigning them as
    //! the initial cluster centers
    srand ((unsigned)time (NULL));

    while (k < numClusters_)
    {
      //! Pick a random pattern
      p = (rand () * numPatterns_ / (RAND_MAX + 1));
      km = patternAssignments_[p];

      //! If pattern is unassigned...
      if (km < 0)
      {
        //! Install the pattern as the first member in the new cluster.
        //! Update the patternAssignments_ vector and pattern-cluster map.
        patternAssignments_[p] = k;
        memClusterPattern_[k][p] = true;

        trainingPatterns_->getPatternRawFeatures (p, valVec);
        trainingPatterns_->getPatternAttributes (p, attribs);

        //! Install affiliation in Clusters array: 
        clusters_->addMember (k, valVec, attribs);

        k++;
      } // if (km < 0)
    } // (while k < numClusters_)
    */

  }


  LIBRARY_API bool kMeans::isMember (int ipat, int kclust)
  {
    return memClusterPattern_[kclust][ipat];
  }


  LIBRARY_API void kMeans::getRanges (double *maxVals, double *minVals)
  {
    trainingPatterns_->getRanges (maxVals, minVals);
  }


  LIBRARY_API int kMeans::recluster ()
  {
    //! Step through all patterns and see if membership should change

    int curClust, newClust, numReassignments = 0;
    int i;
    double *valVec = new double[featureDims_];

    for (i = 0; i < numPatterns_; ++i)
    {
      curClust = patternAssignments_[i];

      //trainingPatterns_->getPatternScaledFeatures(i, valVec);
      trainingPatterns_->getPatternRawFeatures(i, valVec);
      newClust = clusters_->closestCluster(valVec);

      if (curClust != newClust)
      {
        //! Found a closer cluster, perform cluster reassignment
        if (removeMember(i, curClust))
        {
          numReassignments++;
          addMember(i, newClust);
        }
      }
    } // for (i = 0; i < numPatterns_; ++i)

    delete [] valVec;
    return numReassignments;
  }


  LIBRARY_API int kMeans::evalPattern(double *valVec, double *features, double *attribs)
  {
    int knum = clusters_->closestCluster(valVec);
    clusters_->getClusterFeatures(knum, features);
    clusters_->getClusterAttributes(knum, attribs);
    return knum;
  }


  LIBRARY_API int kMeans::evalPattern(vector<double> &valVec, double *features, double *attribs)
  {
    int knum = clusters_->closestCluster(valVec);
    clusters_->getClusterFeatures(knum, features);
    clusters_->getClusterAttributes(knum, attribs);
    return knum;
  }


  LIBRARY_API int kMeans::evalPattern(double *valVec, vector<double> &features, double *attribs)
  {
    int knum = clusters_->closestCluster(valVec);
    clusters_->getClusterFeatures(knum, features);
    clusters_->getClusterAttributes(knum, attribs);
    return knum;
  }


  LIBRARY_API int kMeans::evalPattern(vector<double> &valVec, vector<double> &features, double *attribs)
  {
    int knum = clusters_->closestCluster(valVec);
    clusters_->getClusterFeatures(knum, features);
    clusters_->getClusterAttributes(knum, attribs);
    return knum;
  }


  LIBRARY_API int kMeans::evalPattern(double *valVec, double *features, vector<double> &attribs)
  {
    int knum = clusters_->closestCluster(valVec);
    clusters_->getClusterFeatures(knum, features);
    clusters_->getClusterAttributes(knum, attribs);
    return knum;
  }


  LIBRARY_API int kMeans::evalPattern(vector<double> &valVec, double *features, vector<double> &attribs)
  {
    int knum = clusters_->closestCluster(valVec);
    clusters_->getClusterFeatures(knum, features);
    clusters_->getClusterAttributes(knum, attribs);
    return knum;
  }


  LIBRARY_API int kMeans::evalPattern(double *valVec, vector<double> &features, vector<double> &attribs)
  {
    int knum = clusters_->closestCluster(valVec);
    clusters_->getClusterFeatures(knum, features);
    clusters_->getClusterAttributes(knum, attribs);
    return knum;
  }


  LIBRARY_API int kMeans::evalPattern(vector<double> &valVec, vector<double> &features, vector<double> &attribs)
  {
    int knum = clusters_->closestCluster(valVec);
    clusters_->getClusterFeatures(knum, features);
    clusters_->getClusterAttributes(knum, attribs);
    return knum;
  }


  LIBRARY_API void kMeans::addMember (int ipat, int kclust)
  {
    double *valVec = NULL, *attribs = NULL;

    valVec = new double[featureDims_];
    attribs = new double[attributeDims_];


    //! Verify ipat is not alreadya member of kclust & that kclust has been defined
    if (patternAssignments_[ipat] != kclust)
    {
      //! Pattern is not a member, so install it now.
      trainingPatterns_->getPatternRawFeatures(ipat, valVec);
      trainingPatterns_->getPatternAttributes(ipat, attribs);
      memClusterPattern_[kclust][ipat] = true;

      patternAssignments_[ipat] = kclust;
      clusters_->addMember (kclust, valVec, attribs);
    }

    if (valVec != NULL)
    {
      delete [] valVec;
    }
  }


  LIBRARY_API bool kMeans::removeMember (int ipat, int kclust)
  {
    double *valVec = NULL, *attribs = NULL;
    bool flag = false;

    //! Verify ipat is actually a member of kclust & that kclust has been defined
    if (kclust >= 0) 
    {
      //! The pattern is classified...
      //! but does it belong to kclust?
      if (patternAssignments_[ipat] == kclust)
      {
        valVec = new double[featureDims_];
        attribs = new double[attributeDims_];

        trainingPatterns_->getPatternRawFeatures(ipat, valVec);
        trainingPatterns_->getPatternAttributes(ipat, attribs);

        if (clusters_->removeMember(kclust, valVec, attribs))
        {
          memClusterPattern_[kclust][ipat] = false;
          flag = true;
        }
      } // if (patternAssignments_[ipat] == kclust) 
    } // if (kclust >= 0)

    if (valVec != NULL)
    {
      delete [] valVec;
    }
    return flag;
  }


  LIBRARY_API void kMeans::addTrainingPattern (double *valVec, double *attributes)
  {
    int i, j, k;
    Pattern pat (featureDims_, attributeDims_);
    pat.setRawFeatures (valVec);
    pat.setAttributes (attributes);

    trainingPatterns_->addPattern (pat);
    numPatterns_ = trainingPatterns_->getPatternCount ();

    if (numPatterns_ > 1)
    {
      delete [] patternAssignments_;

      for (i = 0; i < numClusters_; ++i)
      {
        delete [] memClusterPattern_[i];
      }
    }
  
    //! Create vector for pattern assignments to clusters
    patternAssignments_ = new int[numPatterns_];

    for (i = 0; i < numPatterns_; ++i)
    {
      //! Set to "unassigned"
      patternAssignments_[i] = -1;
    }

    //! Allocate space for the membership association matrix
    for (i = 0; i < numClusters_; ++i)
    {
      memClusterPattern_[i] = new bool[numPatterns_];
    }

    //! Assign all patterns to un-associated
    for (j = 0; j < numClusters_; ++j)
    {
      for (k = 0; k < numPatterns_; ++k)
      {
        memClusterPattern_[j][k] = false;
      }
    }
  }


  LIBRARY_API void kMeans::addTrainingPattern(vector<double> &valVec, double *attributes)
  {
    int i, j, k;
    Pattern pat(featureDims_, attributeDims_);
    pat.setRawFeatures(valVec);
    pat.setAttributes(attributes);

    trainingPatterns_->addPattern(pat);
    numPatterns_ = trainingPatterns_->getPatternCount();

    if (numPatterns_ > 1)
    {
      delete[] patternAssignments_;

      for (i = 0; i < numClusters_; ++i)
      {
        delete[] memClusterPattern_[i];
      }
    }

    //! Create vector for pattern assignments to clusters
    patternAssignments_ = new int[numPatterns_];

    for (i = 0; i < numPatterns_; ++i)
    {
      //! Set to "unassigned"
      patternAssignments_[i] = -1;
    }

    //! Allocate space for the membership association matrix
    for (i = 0; i < numClusters_; ++i)
    {
      memClusterPattern_[i] = new bool[numPatterns_];
    }

    //! Assign all patterns to un-associated
    for (j = 0; j < numClusters_; ++j)
    {
      for (k = 0; k < numPatterns_; ++k)
      {
        memClusterPattern_[j][k] = false;
      }
    }
  }


  LIBRARY_API void kMeans::addTrainingPattern(double *valVec, vector<double> &attributes)
  {
    int i, j, k;
    Pattern pat(featureDims_, attributeDims_);
    pat.setRawFeatures(valVec);
    pat.setAttributes(attributes);

    trainingPatterns_->addPattern(pat);
    numPatterns_ = trainingPatterns_->getPatternCount();

    if (numPatterns_ > 1)
    {
      delete[] patternAssignments_;

      for (i = 0; i < numClusters_; ++i)
      {
        delete[] memClusterPattern_[i];
      }
    }

    //! Create vector for pattern assignments to clusters
    patternAssignments_ = new int[numPatterns_];

    for (i = 0; i < numPatterns_; ++i)
    {
      //! Set to "unassigned"
      patternAssignments_[i] = -1;
    }

    //! Allocate space for the membership association matrix
    for (i = 0; i < numClusters_; ++i)
    {
      memClusterPattern_[i] = new bool[numPatterns_];
    }

    //! Assign all patterns to un-associated
    for (j = 0; j < numClusters_; ++j)
    {
      for (k = 0; k < numPatterns_; ++k)
      {
        memClusterPattern_[j][k] = false;
      }
    }
  }


  LIBRARY_API void kMeans::addTrainingPattern(vector<double> &valVec, vector<double> &attributes)
  {
    int i, j, k;
    Pattern pat(featureDims_, attributeDims_);
    pat.setRawFeatures(valVec);
    pat.setAttributes(attributes);

    trainingPatterns_->addPattern(pat);
    numPatterns_ = trainingPatterns_->getPatternCount();

    if (numPatterns_ > 1)
    {
      delete[] patternAssignments_;

      for (i = 0; i < numClusters_; ++i)
      {
        delete[] memClusterPattern_[i];
      }
    }

    //! Create vector for pattern assignments to clusters
    patternAssignments_ = new int[numPatterns_];

    for (i = 0; i < numPatterns_; ++i)
    {
      //! Set to "unassigned"
      patternAssignments_[i] = -1;
    }

    //! Allocate space for the membership association matrix
    for (i = 0; i < numClusters_; ++i)
    {
      memClusterPattern_[i] = new bool[numPatterns_];
    }

    //! Assign all patterns to un-associated
    for (j = 0; j < numClusters_; ++j)
    {
      for (k = 0; k < numPatterns_; ++k)
      {
        memClusterPattern_[j][k] = false;
      }
    }
  }


  LIBRARY_API void kMeans::clearTrainingPatterns ()
  {
    trainingPatterns_->clearPatterns ();
  }


  LIBRARY_API Clusters& kMeans::getClusters()
  {
    return *clusters_;
  }


  LIBRARY_API bool kMeans::getClusterInfo (int c, double *features, double *attributes)
  {
    if (c < numClusters_ && c >= 0)
    {
      clusters_->getClusterFeatures(c, features);
      clusters_->getClusterAttributes (c, attributes);
      return true;
    }
    return false;
  }


  LIBRARY_API bool kMeans::getClusterInfo(int c, vector<double> &features, double *attributes)
  {
    if (c < numClusters_ && c >= 0)
    {
      clusters_->getClusterFeatures(c, features);
      clusters_->getClusterAttributes(c, attributes);
      return true;
    }
    return false;
  }


  LIBRARY_API bool kMeans::getClusterInfo(int c, double *features, vector<double> &attributes)
  {
    if (c < numClusters_ && c >= 0)
    {
      clusters_->getClusterFeatures(c, features);
      clusters_->getClusterAttributes(c, attributes);
      return true;
    }
    return false;
  }


  LIBRARY_API bool kMeans::getClusterInfo(int c, vector<double> &features, vector<double> &attributes)
  {
    if (c < numClusters_ && c >= 0)
    {
      clusters_->getClusterFeatures(c, features);
      clusters_->getClusterAttributes(c, attributes);
      return true;
    }
    return false;
  }

} // Clustering

