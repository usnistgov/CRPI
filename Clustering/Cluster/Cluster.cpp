///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Clustering
//  Workfile:        Cluster.cpp
//  Revision:        1.0 - 28 September, 2009
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Cluster type and collections container class declarations.
//
///////////////////////////////////////////////////////////////////////////////

#include "Cluster.h"
#include <math.h>

namespace Clustering
{
  /////////////////////////////////////////////////////////////////////////////
  //                          CLUSTER DEFINITIONS                            //
  /////////////////////////////////////////////////////////////////////////////

  LIBRARY_API Cluster::Cluster(int fdims, int adims) :
    members_(0),
    dimensions_(fdims),
    attributeDims_(adims),
    minMembers_(1)
  {
    unsigned int i;
    features_.clear();

    for (i = 0; i < dimensions_; ++i)
    {
      features_.push_back(0.0f);
    }

    avgAttributes_.clear();
    for (i = 0; i < attributeDims_; ++i)
    {
      avgAttributes_.push_back(0.0f);
    }
  }


  LIBRARY_API Cluster::~Cluster ()
  {
    features_.clear();
    avgAttributes_.clear();
  }


  LIBRARY_API void Cluster::setMinMembers(int min)
  {
    minMembers_ = min;
  }


  LIBRARY_API void Cluster::addMember (double *valVec, double *attributes)
  {
    unsigned int i;

    //! Add the influence of the new pattern on the cluster centroid
    for (i = 0; i < dimensions_; ++i)
    {
      features_[i] = (features_[i] * members_ + valVec[i]) / ((double)members_ + 1);
    }
    for (i = 0; i < attributeDims_; ++i)
    {
      avgAttributes_.at(i) = (avgAttributes_.at(i) * members_ + attributes[i]) / ((double)members_ + 1);
    }
    ++members_;
  }


  LIBRARY_API void Cluster::addMember(vector<double> &valVec, double *attributes)
  {
    unsigned int i;

    //! Add the influence of the new pattern on the cluster centroid
    for (i = 0; i < dimensions_; ++i)
    {
      features_[i] = (features_[i] * members_ + valVec.at(i)) / ((double)members_ + 1);
    }
    for (i = 0; i < attributeDims_; ++i)
    {
      avgAttributes_.at(i) = (avgAttributes_.at(i) * members_ + attributes[i]) / ((double)members_ + 1);
    }
    ++members_;
  }


  LIBRARY_API void Cluster::addMember(double *valVec, vector<double> &attributes)
  {
    unsigned int i;

    //! Add the influence of the new pattern on the cluster centroid
    for (i = 0; i < dimensions_; ++i)
    {
      features_[i] = (features_[i] * members_ + valVec[i]) / ((double)members_ + 1);
    }
    for (i = 0; i < attributeDims_; ++i)
    {
      avgAttributes_.at(i) = (avgAttributes_.at(i) * members_ + attributes.at(i)) / ((double)members_ + 1);
    }
    ++members_;
  }


  LIBRARY_API void Cluster::addMember(vector<double> &valVec,vector<double> &attributes)
  {
    unsigned int i;

    //! Add the influence of the new pattern on the cluster centroid
    for (i = 0; i < dimensions_; ++i)
    {
      features_[i] = (features_[i] * members_ + valVec.at(i)) / ((double)members_ + 1);
    }
    for (i = 0; i < attributeDims_; ++i)
    {
      avgAttributes_.at(i) = (avgAttributes_.at(i) * members_ + attributes.at(i)) / ((double)members_ + 1);
    }
    ++members_;
  }


  LIBRARY_API bool Cluster::removeMember (double *valVec, double *attributes)
  {
    unsigned int i;

    if (members_ <= minMembers_)
    {
      //! Cannot go below minimum
      return false;
    }

    //! Remove the influence of the new pattern from the cluster centroid
    for (i = 0; i < dimensions_; ++i)
    {
      features_.at(i) = (features_.at(i) * members_ - valVec[i]) / ((double)members_ - 1);
    }

    for (i = 0; i < attributeDims_; ++i)
    {
      avgAttributes_.at(i) = (avgAttributes_.at(i) * members_ - attributes[i]) / ((double)members_ - 1);
    }
    --members_;
    return true;
  }


  LIBRARY_API bool Cluster::removeMember(vector<double> &valVec, double *attributes)
  {
    unsigned int i;

    if (members_ <= minMembers_)
    {
      //! Cannot go below minimum
      return false;
    }

    //! Remove the influence of the new pattern from the cluster centroid
    for (i = 0; i < dimensions_; ++i)
    {
      features_.at(i) = (features_.at(i) * members_ - valVec.at(i)) / ((double)members_ - 1);
    }

    for (i = 0; i < attributeDims_; ++i)
    {
      avgAttributes_.at(i) = (avgAttributes_.at(i) * members_ - attributes[i]) / ((double)members_ - 1);
    }
    --members_;
    return true;
  }


  LIBRARY_API bool Cluster::removeMember(double *valVec, vector<double> &attributes)
  {
    unsigned int i;

    if (members_ <= minMembers_)
    {
      //! Cannot go below minimum
      return false;
    }

    //! Remove the influence of the new pattern from the cluster centroid
    for (i = 0; i < dimensions_; ++i)
    {
      features_.at(i) = (features_.at(i) * members_ - valVec[i]) / ((double)members_ - 1);
    }

    for (i = 0; i < attributeDims_; ++i)
    {
      avgAttributes_.at(i) = (avgAttributes_.at(i) * members_ - attributes[i]) / ((double)members_ - 1);
    }
    --members_;
    return true;
  }


  LIBRARY_API bool Cluster::removeMember(vector<double> &valVec, vector<double> &attributes)
  {
    unsigned int i;

    if (members_ <= minMembers_)
    {
      //! Cannot go below minimum
      return true;
    }

    //! Remove the influence of the new pattern from the cluster centroid
    for (i = 0; i < dimensions_; ++i)
    {
      features_.at(i) = (features_.at(i) * members_ - valVec.at(i)) / ((double)members_ - 1);
    }

    for (i = 0; i < attributeDims_; ++i)
    {
      avgAttributes_.at(i) = (avgAttributes_.at(i) * members_ - attributes.at(i)) / ((double)members_ - 1);
    }
    --members_;
    return false;
  }


  LIBRARY_API void Cluster::getAttributes (double *valVec)
  {
    unsigned int i;
    for (i = 0; i < attributeDims_; ++i)
    {
      valVec[i] = avgAttributes_.at(i);
    }
  }


  LIBRARY_API void Cluster::getAttributes (vector<double> &valVec)
  {
    valVec.clear();
    for (int i = 0; i < attributeDims_; ++i)
    {
      valVec.push_back(avgAttributes_.at(i));
    }
  }


  LIBRARY_API void Cluster::getFeatures (double *valVec)
  {
    unsigned int i;
    for (i = 0; i < dimensions_; ++i)
    {
      valVec[i] = features_.at(i);
    }
  }


  LIBRARY_API void Cluster::getFeatures (vector<double> &valVec)
  {
    unsigned int i;
    valVec.clear();
    for (i = 0; i < dimensions_; ++i)
    {
      valVec.push_back(features_.at(i));
    }
  }


  LIBRARY_API double Cluster::distance (double *valVec)
  {
    double dist = 0.0f;
    unsigned int i;

    //! Compute the Euclidean distance from valVec to the cluster center
    for (i = 0; i < dimensions_; ++i)
    {
      dist += (valVec[i] - features_.at(i)) * (valVec[i] - features_.at(i));
    }
    dist = sqrt (dist);
    return dist;
  }


  LIBRARY_API double Cluster::distance(vector<double> &valVec)
  {
    double dist = 0.0f;
    unsigned int i;

    //! Compute the Euclidean distance from valVec to the cluster center
    for (i = 0; i < dimensions_; ++i)
    {
      dist += (valVec.at(i) - features_.at(i)) * (valVec.at(i) - features_.at(i));
    }
    dist = sqrt(dist);
    return dist;
  }


  LIBRARY_API int Cluster::getFeatureDimensions ()
  {
    return dimensions_;
  }


  LIBRARY_API int Cluster::getAttributeDimensions ()
  {
    return attributeDims_;
  }


  LIBRARY_API void Cluster::resize (int fdim, int adim)
  {
    unsigned int i;
    features_.clear();
    dimensions_ = fdim;
    features_.resize(dimensions_);

    for (i = 0; i < dimensions_; ++i)
    {
      features_.at(i) = 0.0f;
    }

    avgAttributes_.clear();
    attributeDims_ = adim;
    avgAttributes_.resize(attributeDims_);
    for (i = 0; i < attributeDims_; ++i)
    {
      avgAttributes_.at(i) = 0.0f;
    }
  }

  
  LIBRARY_API int Cluster::size()
  {
    return members_;
  }


  /////////////////////////////////////////////////////////////////////////////
  //                    CLUSTER COLLECTION DEFINITIONS                       //
  /////////////////////////////////////////////////////////////////////////////

  LIBRARY_API Clusters::Clusters (int kclusts, int fdim, int adim):
    minMembers_(0)
  {
    resize (kclusts, fdim, adim);
  }


  LIBRARY_API Clusters::~Clusters ()
  {
    clusters_.clear ();
  }


  LIBRARY_API void Clusters::resize (int kclust, int fdim, int adim)
  {
    Cluster *temp;
    int i;

    clusters_.clear ();
    for (i = 0; i < kclust; ++i)
    {
      temp = new Cluster(fdim, adim);
      temp->setMinMembers(minMembers_);
      clusters_.push_back (*temp);
      temp = NULL;
    }
    kClusters_ = kclust;
  }


  LIBRARY_API void Clusters::setMinMembers(int min)
  {
    minMembers_ = min;
    for (int i = 0; i < clusters_.size(); ++i)
    {
      clusters_.at(i).setMinMembers(min);
    }
  }


  LIBRARY_API void Clusters::addMember (int kclust,
                                        double *valVec,
                                        double *attributes)
  {
    //! Adjust the cluster with the feature values in the specified cluster
    if (kclust < kClusters_ && kclust >= 0)
    {
      clusters_.at(kclust).addMember (valVec, attributes);
    }
  }


  LIBRARY_API void Clusters::addMember(int kclust,
                                       vector<double> &valVec,
                                       double *attributes)
  {
    //! Adjust the cluster with the feature values in the specified cluster
    if (kclust < kClusters_ && kclust >= 0)
    {
      clusters_.at(kclust).addMember(valVec, attributes);
    }
  }


  LIBRARY_API void Clusters::addMember(int kclust,
                                       double *valVec,
                                       vector<double> &attributes)
  {
    //! Adjust the cluster with the feature values in the specified cluster
    if (kclust < kClusters_ && kclust >= 0)
    {
      clusters_.at(kclust).addMember(valVec, attributes);
    }
  }


  LIBRARY_API void Clusters::addMember(int kclust,
                                       vector<double> &valVec,
                                       vector<double> &attributes)
  {
    //! Adjust the cluster with the feature values in the specified cluster
    if (kclust < kClusters_ && kclust >= 0)
    {
      clusters_.at(kclust).addMember(valVec, attributes);
    }
  }


  LIBRARY_API bool Clusters::removeMember (int kclust,
                                           double *valVec,
                                           double *attributes)
  {
    //! Adjust the cluster by removing the feature values from the specified
    //! cluster
    if (kclust < kClusters_)
    {
      return clusters_.at(kclust).removeMember (valVec, attributes);
    }
    return false;
  }


  LIBRARY_API bool Clusters::removeMember(int kclust,
                                          vector<double> &valVec,
                                          double *attributes)
  {
    //! Adjust the cluster by removing the feature values from the specified
    //! cluster
    if (kclust < kClusters_)
    {
      return clusters_.at(kclust).removeMember(valVec, attributes);
    }
    return false;
  }


  LIBRARY_API bool Clusters::removeMember(int kclust,
                                          double *valVec,
                                          vector<double> &attributes)
  {
    //! Adjust the cluster by removing the feature values from the specified
    //! cluster
    if (kclust < kClusters_)
    {
      return clusters_.at(kclust).removeMember(valVec, attributes);
    }
    return false;
  }


  LIBRARY_API bool Clusters::removeMember(int kclust,
                                          vector<double> &valVec,
                                          vector<double> &attributes)
  {
    //! Adjust the cluster by removing the feature values from the specified
    //! cluster
    if (kclust < kClusters_)
    {
      return clusters_.at(kclust).removeMember(valVec, attributes);
    }
    return false;
  }


  LIBRARY_API Cluster& Clusters::at(unsigned int kclust)
  {
    return clusters_.at(kclust);
  }


  LIBRARY_API int Clusters::closestCluster (double *valVec)
  {
    double minDist;
    double testDist;

    int k_closest = 0, i;

    minDist = clusters_.at(0).distance (valVec);

    for (i = 1; i < kClusters_; ++i)
    {
      testDist = clusters_.at(i).distance (valVec);
      if (testDist < minDist)
      {
        minDist = testDist;
        k_closest = i;
      }
    }
    return k_closest;
  }


  LIBRARY_API int Clusters::closestCluster(vector<double> &valVec)
  {
    double minDist;
    double testDist;

    int k_closest = 0, i;

    minDist = clusters_.at(0).distance(valVec);

    for (i = 1; i < kClusters_; ++i)
    {
      testDist = clusters_.at(i).distance(valVec);
      if (testDist < minDist)
      {
        minDist = testDist;
        k_closest = i;
      }
    }
    return k_closest;
  }


  LIBRARY_API int Clusters::getNumClusters ()
  {
    return kClusters_;
  }


  LIBRARY_API bool Clusters::getClusterAttributes (int kclust, double *attributes)
  {
    if (kclust < kClusters_ && kclust >= 0)
    {
      clusters_.at(kclust).getAttributes(attributes);
      return true;
    }
    return false;
  }


  LIBRARY_API bool Clusters::getClusterAttributes (int kclust, vector<double> &attributes)
  {
    if (kclust < kClusters_ && kclust >= 0)
    {
      clusters_.at(kclust).getAttributes(attributes);
      return true;
    }
    return false;
  }


  LIBRARY_API bool Clusters::getClusterFeatures (int kclust, double *features)
  {
    if (kclust < kClusters_)
    {
      clusters_.at(kclust).getFeatures (features);
      return true;
    }
    return false;
  }

  LIBRARY_API bool Clusters::getClusterFeatures (int kclust, vector<double> &features)
  {
    if (kclust < kClusters_)
    {
      clusters_.at(kclust).getFeatures(features);
      return true;
    }
    return false;
  }

} // Clustering