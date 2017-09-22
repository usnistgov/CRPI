///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Clustering
//  Workfile:        Cluster.h
//  Revision:        1.0 - 28 September, 2009
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Cluster type and collections container class definitions.  Assumes feature
//  vectors are of type double.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef CLUSTER_H
#define CLUSTER_H

#include <vector>
#include "../../portable.h"

using namespace std;

namespace Clustering
{
  //! @ingroup Clustering
  //!
  //! @brief Cluster type class description
  //!
  class LIBRARY_API Cluster
  {
  public:
    //! @brief Constructor
    //!
    //! @param fdims The number of dimensions for the feature vector
    //! @param adims The number of dimensions for the attribute vector
    //!
    Cluster (int fdims, int adims);

    //! @brief Default destructor
    //!
    ~Cluster ();

    //! @brief Set the minimum number of members required for a cluster
    //!
    //! @param val The minimum number of members required for a cluster
    //!
    void setMinMembers(int min);

    //! @brief Add a member to this cluster
    //!
    //! @param valVec     Pattern feature vector being added to the cluster centroid
    //! @param attributes Pattern attributes being added to the cluster's definition
    //!
    void addMember(double *valVec, double *attributes);
    void addMember(vector<double> &valVec, double *attributes);
    void addMember(double *valVec, vector<double> &attributes);
    void addMember(vector<double> &valVec, vector<double> &attributes);

    //! @brief Remove a member from this cluster
    //!
    //! @param valVec    Pattern feature vector being removed from the cluster centroid
    //! @param attribute Pattern attributes being removed from the cluster's definition
    //!
    //! @return True if the process completed successfully, false otherwise
    //!
    bool removeMember (double *valVec, double *attributes);
    bool removeMember (vector<double> &valVec, double *attributes);
    bool removeMember (double *valVec, vector<double> &attributes);
    bool removeMember (vector<double> &valVec, vector<double> &attributes);

    //! @brief Populate a vector with a copy of the cluster centroid
    //!
    //! @param valVec Pointer to the vector to be filled by this function call
    //!
    void getFeatures (double *valVec);
    void getFeatures (vector<double> &valVec);

    //! @brief Access the cluster attribute
    //!
    //! @param valVec The populated array with the averages of the attributes of the cluster
    //!
    void getAttributes (double *valVec);
    void getAttributes (vector<double> &valVec);

    //! @brief Get the distance between a query pattern and the cluster center
    //!
    //! @param valVec Query pattern
    //!
    //! @return The Euclidean distance
    //!
    double distance (double *valVec);
    double distance (vector<double> &valVec);

    //! @brief Get the size of a feature vector
    //!
    //! @return The size of the feature vector
    //!
    int getFeatureDimensions ();

    //! @brief Get the size of an attribute vector
    //!
    //! @return The size of the attribute vector
    //!
    int getAttributeDimensions ();

    //! @brief Adjust the size of a feature vector
    //!
    //! @param fdim The new feature vector dimensions
    //! @param adim The new attribute vector dimensions
    //!
    void resize (int fdim, int adim);

    //! @brief Get the number of member patterns currently in this cluster
    //!
    //! @return The number of member patterns influencing the cluster centroid
    //!
    int size();

  private:
    //! @brief Average of the attributes over all cluster members
    //!
    vector<double> avgAttributes_;

    //! @brief Number of member patterns in this cluster
    //!
    int members_;

    //! @brief Mimimum number of member patterns required for a cluster (default is 0)
    //!
    int minMembers_;

    //! @brief Vector of cluster archetype (centroid)
    //!
    vector<double> features_;

    //! @brief Feature dimensions
    //!
    unsigned int dimensions_;

    //! @brief Attribute dimensions
    //!
    unsigned int attributeDims_;
  }; // Cluster



  //! @ingroup Clustering
  //!
  //! @brief Cluster collection class description
  //!
  class LIBRARY_API Clusters
  {
  public:

    //! @brief Constructor
    //!
    //! @param kclusts The number of clusters to be in the collection
    //! @param fdim    The number of values in each feature vector
    //! @param adim    The number of values in each attribute vector
    //!
    Clusters (int kclusts, int fdim, int adim);

    //! @brief Default destructor
    //!    
    ~Clusters ();

    //! @brief Adjust the number of clusters
    //!
    //! @param kclust The number of clusters to be in the collection
    //! @param fdim   The number of values in each feature vector
    //! @Param adim   The number of values in each attribute vector
    //!
    void resize (int kclust, int fdim, int adim);

    //! @brief Set the minimum number of members required for a cluster
    //!
    //! @param val The minimum number of members required for a cluster
    //!
    void setMinMembers(int min);

    //! @brief Find the cluster closest to the query feature vector
    //!
    //! @param valVec Query feature vector looking for a cluster to call home
    //!
    //! @return The index of the cluster closest to valVec
    //!
    int closestCluster (double *valVec);
    int closestCluster (vector<double> &valVec);

    //! @brief Install a member into the cluster
    //!
    //! @param kclus      The cluster number in which the new values are to be
    //!                   added
    //! @param valVec     The feature vector to be factored into the cluster
    //! @param attributes The attribute to be factored into the cluster
    //!
    void addMember (int kclust, double *valVec, double *attributes);
    void addMember (int kclust, vector<double> &valVec, double *attributes);
    void addMember (int kclust, double *valVec, vector<double> &attributes);
    void addMember (int kclust, vector<double> &valVec, vector<double> &attributes);

    //! @brief Remove a member from the cluster
    //!
    //! @param kclust     The cluster number from which values are to be removed
    //! @param valVec     The feature vector to be factored out of the cluster
    //! @param attributes The attribute vector to be factored out of the cluster
    //!
    //! @return True if the process completed successfully, false otherwise
    //!
    bool removeMember (int kclust, double *valVec, double *attributes);
    bool removeMember (int kclust, vector<double> &valVec, double *attributes);
    bool removeMember (int kclust, double *valVec, vector<double> &attributes);
    bool removeMember (int kclust, vector<double> &valVec, vector<double> &attributes);

    //! @brief Access a specific cluster at a specific index
    //!
    //! @param kclust The specified cluster index number
    //!
    //! @return A pointer to the indicated cluster at index kclust
    //!
    Cluster& at(unsigned int kclust);

    //! @brief Get the number of clusters in the population
    //!
    //! @return The number of unique clusters currently defined
    //!
    int getNumClusters ();

    //! @brief Get the average attribute of the specified cluster
    //!
    //! @param kclust     The cluster being queried
    //! @Param attributes The average attribute cluster for the queried cluster
    //!
    //! @return True if the cluster exists, false otherwise
    //!
    bool getClusterAttributes (int kclust, double *attributes);
    bool getClusterAttributes (int kclust, vector<double> &attributes);

    //! @brief Get the cluster centroid
    //!
    //! @param kclust   The Cluster being queried
    //! @param features The feature centroid of the specified cluster
    //!
    //! @return True if the cluster exists, false otherwise
    //!
    bool getClusterFeatures (int kclust, double *features);
    bool getClusterFeatures (int kclust, vector<double> &features);

  private:

    //! @brief The collection of clusters defined
    //!
    vector<Cluster> clusters_;

    //! @brief The number of clusters in the collection
    //!
    int kClusters_;

    //! @brief The mimimum number of member patterns required for a given cluster (default is 0)
    //!
    int minMembers_;
  }; // Clusters
} // Clustering

#endif