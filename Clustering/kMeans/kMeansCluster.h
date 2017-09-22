///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Clustering
//  Workfile:        kMeansCluster.h
//  Revision:        1.0 - 28 September, 2009
//  Author:          J. Marvel
//
//  Description
//  ===========
//  K-Means clustering interface declaration file.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef KMEANSCLUSTER_H
#define KMEANSCLUSTER_H

#include "../Patterns/Pattern.h"
#include "../Cluster/Cluster.h"

namespace Clustering
{
  //! @ingroup Clustering
  //!
  //! @brief Container class for both Patterns and Clusters, and driver
  //!        interface for adjusting the two.
  //!
  class LIBRARY_API kMeans
  {
  public:
    //! @brief Constructor
    //!
    //! @param fdim   The number of elements in a feature vector
    //! @Param adim   The number of elements in an attribute vector
    //! @param kclust The number of clusters to be defined
    //! @param path   Path to the file containing feature vector values
    //!
    kMeans (int fdim, int adim, int kclust, char *path);

    //! @brief Default destructor
    //!
    ~kMeans ();

    //! @brief Set the minimum number of members required for a given cluster
    //!
    //! @param val The minimum number of members required for a cluster
    //!
    void setMinClusterMembers(int min);

    //! @brief Seed the clusters with random patterns
    //!
    void seedClusters ();

    //! @brief Run the re-cluster method to adjust the cluster definitions
    //!        and reassign patterns to clusters that better fit their
    //!        profiles
    //!
    //! @return The number of patterns reassigned
    //!
    int recluster ();
    
    //! @brief Evaluate an input pattern and get the value of the best fit's
    //!        attribute
    //!
    //! @param valVec   The input pattern to be sampled
    //! @param features The centroid of the closest cluster
    //! @param attribs  The average attributes of the closest cluster
    //!
    //! @return The id of the closest cluster
    //!
    int evalPattern (double *valVec, double *features, double *attribs);
    int evalPattern (vector<double> &valVec, double *features, double *attribs);
    int evalPattern (double *valVec, vector<double> &features, double *attribs);
    int evalPattern (vector<double> &valVec, vector<double> &features, double *attribs);
    int evalPattern (double *valVec, double *features, vector<double> &attribs);
    int evalPattern (vector<double> &valVec, double *features, vector<double> &attribs);
    int evalPattern (double *valVec, vector<double> &features, vector<double> &attribs);
    int evalPattern (vector<double> &valVec, vector<double> &features, vector<double> &attribs);

    //! @brief Test whether or not a known pattern is a member of a particular
    //!        cluster
    //!
    //! @param ipat   The pattern index to be tested for membership
    //! @param kclust The cluster index that we are asking if ipat is a member of
    //!
    //! @return True if ipat is part of kclust, false otherwise
    //!
    bool isMember (int ipat, int kclust);

    //! @brief Grab the features and attribute of a specified cluster
    //!
    //! @param c          The cluster number being accessed
    //! @param features   The cluster feature centroid
    //! @Param attributes The cluster attribute vector
    //!
    //! @return True if the specified cluster exists, false otherwise
    //!
    bool getClusterInfo (int c, double *features, double *attributes);
    bool getClusterInfo (int c, vector<double> &features, double *attributes);
    bool getClusterInfo (int c, double *features, vector<double> &attributes);
    bool getClusterInfo (int c, vector<double> &features, vector<double> &attributes);

    //! @brief Add a specified pattern to a specified cluster
    //!
    //! @param ipat   The index number of the pattern to be added
    //! @param kclust The index number of the cluster being adjusted
    //!
    void addMember (int ipat, int kclust);

    //! @brief Remove a specified pattern from a specified cluster
    //!
    //! @param ipat   The index number of the pattern to be removed
    //! @param kclust The index number of the cluster being adjusted
    //!
    //! @return True if the process completes successfully, false otherwise
    //!
    bool removeMember (int ipat, int kclust);

    //! @brief Get the rage of input pattern values
    //!
    //! @param maxVals The ceiling of all x pattern values returned
    //! @param minVals The floor of all x pattern values returned
    //!
    void getRanges (double *maxVals, double *minVals);

    //! @brief Add a new pattern to the collection of patterns
    //!
    //! @param valVec     The pattern vector being added to the collection
    //! @param attributes The attributes of the feature vector
    //!
    void addTrainingPattern (double *valVec, double *attribute);
    void addTrainingPattern (vector<double> &valVec, double *attribute);
    void addTrainingPattern (double *valVec, vector<double> &attribute);
    void addTrainingPattern (vector<double> &valVec, vector<double> &attribute);

    //! @brief Remove all training patterns from the cluster population
    //!
    void clearTrainingPatterns ();

    //! @brief Access the raw cluster data
    //!
    //! @return A pointer to the Clusters object
    //!
    Clusters& getClusters();

  private:

    //! @brief Collection of patterns used for clustering
    //!
    Patterns *trainingPatterns_;

    //! @brief Collection of clusters used in clustering implementation
    //!
    Clusters *clusters_; 

    //! @brief The number of patterns in the system
    //!
    int numPatterns_;

    //! @brief The number of clusters being maintained
    //!
    int numClusters_;

    //! @brief The number of elements in the feature vector
    //!
    int featureDims_;

    //! @brief The number of elements in the attribute vector
    //!
    int attributeDims_;

    //! @brief Matrix used to cross-check pattern-cluster affiliations
    //!
    bool **memClusterPattern_;

    //! @brief Cluster assignment vector for each pattern
    //!
    int *patternAssignments_;

    //! @brief The minimum number of members required for a given cluster (default is 0)
    //!
    int minClusterMembers_;
  }; // kMeans
} // Clustering

#endif