///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Registration Toolkit
//  Workfile:        CoordFrameReg.cpp
//  Revision:        25 August, 2015
//  Author:          J. Marvel
//
//  Description
//  ===========
// 
///////////////////////////////////////////////////////////////////////////////

#include "CoordFrameReg.h"

//! Clustering
#include "kMeansCluster.h"

using namespace Clustering;

namespace Registration
{
  LIBRARY_API bool reg2target(vector<point> &sutPoints, vector<point> &tarPoints, matrix &out)
  {
    double precision = 0.001f;
    int i;
    point temp;
    point sut_p[3], tar_p[3];
    point sut_v1, sut_v2, tar_v1, tar_v2;
    point sut_x_hat, tar_x_hat;
    double mag;
    point sut_z_hat, tar_z_hat;
    point sut_y_hat, tar_y_hat;

    matrix test(4, 4);

    matrix sut_H(4, 4), tar_H(4, 4), sut_H_inv(4, 4);

    vector<point>::iterator sut_iter, tar_iter;

    if (sutPoints.size() != tarPoints.size() || sutPoints.size() < 3 ||
      out.cols != 4 || out.rows != 4)
    {
      //! Dimensions are wrong
      return false;
    }

    sut_iter = sutPoints.begin();
    tar_iter = tarPoints.begin();

    //! Take the first three points, discard the rest
    for (i = 0; i < 3; ++i, ++sut_iter, ++tar_iter)
    {
      sut_p[i] = *sut_iter;
      tar_p[i] = *tar_iter;

      //cout << sut_p[i].x << " " << sut_p[i].y << " " << sut_p[i].z << "  |  " << tar_p[i].x << " " << tar_p[i].y << " " << tar_p[i].z << endl;
    }

    sut_v1 = sut_p[1] - sut_p[0];
    sut_v2 = sut_p[2] - sut_p[0];

    //! Test for collinearity
    temp = sut_v1.cross(sut_v2);
    if (fabs(temp.x) <= precision && fabs(temp.y) <= precision && fabs(temp.z) <= precision)
    {
      //! Points are collinear.
      return false;
    }

    tar_v1 = tar_p[1] - tar_p[0];
    tar_v2 = tar_p[2] - tar_p[0];

    mag = sut_v1.magnitude();
    if (mag < precision)
    {
      return false;
    }
    sut_x_hat = sut_v1 / mag;

    mag = tar_v1.magnitude();
    if (mag < precision)
    {
      return false;
    }
    tar_x_hat = tar_v1 / mag;

    temp = sut_x_hat.cross(sut_v2);
    mag = temp.magnitude();
    if (mag < precision)
    {
      return false;
    }
    sut_z_hat = temp / mag;

    temp = tar_x_hat.cross(tar_v2);
    mag = temp.magnitude();
    if (mag < precision)
    {
      return false;
    }
    tar_z_hat = temp / mag;

    sut_y_hat = sut_x_hat.cross(sut_z_hat);
    tar_y_hat = tar_x_hat.cross(tar_z_hat);

    sut_H.at(0, 0) = sut_x_hat.x;
    sut_H.at(1, 0) = sut_x_hat.y;
    sut_H.at(2, 0) = sut_x_hat.z;
    sut_H.at(0, 1) = sut_y_hat.x;
    sut_H.at(1, 1) = sut_y_hat.y;
    sut_H.at(2, 1) = sut_y_hat.z;
    sut_H.at(0, 2) = sut_z_hat.x;
    sut_H.at(1, 2) = sut_z_hat.y;
    sut_H.at(2, 2) = sut_z_hat.z;
    sut_H.at(0, 3) = sut_p[0].x;
    sut_H.at(1, 3) = sut_p[0].y;
    sut_H.at(2, 3) = sut_p[0].z;
    sut_H.at(3, 3) = 1.0f;

    tar_H.at(0, 0) = tar_x_hat.x;
    tar_H.at(1, 0) = tar_x_hat.y;
    tar_H.at(2, 0) = tar_x_hat.z;
    tar_H.at(0, 1) = tar_y_hat.x;
    tar_H.at(1, 1) = tar_y_hat.y;
    tar_H.at(2, 1) = tar_y_hat.z;
    tar_H.at(0, 2) = tar_z_hat.x;
    tar_H.at(1, 2) = tar_z_hat.y;
    tar_H.at(2, 2) = tar_z_hat.z;
    tar_H.at(0, 3) = tar_p[0].x;
    tar_H.at(1, 3) = tar_p[0].y;
    tar_H.at(2, 3) = tar_p[0].z;
    tar_H.at(3, 3) = 1.0f;

    sut_H_inv = sut_H.inv();

    out = tar_H * sut_H_inv;

    return true;
  }


  LIBRARY_API bool reg2targetML(vector<point> &sutPoints,
                                vector<point> &tarPoints,
                                int numRegs,
                                vector<point> &kernels,
                                vector<matrix> &outs)
  {
    kMeans world_clusters(3, 3, numRegs, NULL);
    world_clusters.setMinClusterMembers(1);
    int count = 0;

    if (sutPoints.size() != tarPoints.size())
    {
      return false;
    }

    point  wrd, rob, centerf, centera;
    vector<point> world;
    vector<point> robot;
    matrix r_2_w(4, 4), w_2_r(4, 4);
    outs.resize(numRegs);
    for (count = 0; count < numRegs; ++count)
    {
      outs.at(count).resize(4, 4);
    }
    vector<double> feat, attrib;
    vector<point>::iterator iter_sut, iter_tar;

    iter_sut = sutPoints.begin();
    iter_tar = tarPoints.begin();

    count = 0;
    for (; iter_sut != sutPoints.end(); ++iter_sut, ++iter_tar, ++count)
    {
      feat.clear();
      attrib.clear();

      //! Features are the points in the source coordinate system
      feat.push_back(iter_sut->x);
      feat.push_back(iter_sut->y);
      feat.push_back(iter_sut->z);

      //! Attributes are the points in the target coordinate system
      attrib.push_back(iter_tar->x);
      attrib.push_back(iter_tar->y);
      attrib.push_back(iter_tar->z);
#ifdef NOISY
      //! Add training pattern to cluster space
      cout << "Add pattern ((" << feat.at(0) << ", " << feat.at(1) << ", " << feat.at(2) << "), ("
        << attrib.at(0) << ", " << attrib.at(1) << ", " << attrib.at(2) << "}}" << endl;
#endif
      world_clusters.addTrainingPattern(feat, attrib);
    }
#ifdef NOISY
    cout << count << " patterns added to the collection of clusters" << endl;
    cout << "Seeding clusters" << endl;
#endif

    world_clusters.seedClusters();
    count = 0;
    do
    {
      count = world_clusters.recluster();
#ifdef NOISY
      cout << count << " patterns moved." << endl;
#endif
    } while (count > 0);

#ifdef NOISY
    cout << "Finished clustering." << endl;
#endif

#ifdef NOISY
    cout << "Generating robot-world registration data" << endl;
#endif

    kernels.clear();
    for (int i = 0; i < numRegs; ++i)
    {
      world_clusters.getClusterInfo(i, feat, attrib);
#ifdef NOISY
      cout << "Cluster data: " << i << " ((" << feat.at(0) << ", " << feat.at(1) << ", " << feat.at(2) << "), ("
        << attrib.at(0) << ", " << attrib.at(1) << ", " << attrib.at(2) << ")) with "
        << world_clusters.getClusters().at(i).size() << " members" << endl;
#endif
      centerf.x = feat.at(0);
      centerf.y = feat.at(1);
      centerf.z = feat.at(2);
      centera.x = attrib.at(0);
      centera.y = attrib.at(1);
      centera.z = attrib.at(2);
      kernels.push_back(centera);

#ifdef NOISY
      for (int j = 0; j < truth.size(); ++j)
      {
        feat.at(0) = sutPoints.at(j).x;
        feat.at(1) = sutPoints.at(j).y;
        feat.at(2) = sutPoints.at(j).z;
        attrib.at(0) = tarPoints.at(j).x;
        attrib.at(1) = tarPoints.at(j).y;
        attrib.at(2) = tarPoints.at(j).z;
        if (world_clusters.getClusters().closestCluster(feat) == i)
        {
          cout << "  Member : ((" << feat.at(0) << ", " << feat.at(1) << ", " << feat.at(2) << "), ("
            << attrib.at(0) << ", " << attrib.at(1) << ", " << attrib.at(2) << "))" << endl;
        }
      } // for (int j = 0; j < truth.size(); ++j)
      cout << endl;
#endif
      world.clear();
      robot.clear();
      world.resize(3);
      robot.resize(3);

      outs.at(i).setAll(0.0f);

      world.at(0) = centera;
      robot.at(0) = centerf;
      count = 0;
      for (int j = 0; j < tarPoints.size(); ++j)
      {
        wrd.x = tarPoints.at(j).x;
        wrd.y = tarPoints.at(j).y;
        wrd.z = tarPoints.at(j).z;
        world.at(1) = wrd;
        rob.x = sutPoints.at(j).x;
        rob.y = sutPoints.at(j).y;
        rob.z = sutPoints.at(j).z;
        robot.at(1) = rob;

        for (int k = j + 1; k < tarPoints.size(); ++k)
        {
          wrd.x = tarPoints.at(k).x;
          wrd.y = tarPoints.at(k).y;
          wrd.z = tarPoints.at(k).z;
          world.at(2) = wrd;
          rob.x = sutPoints.at(k).x;
          rob.y = sutPoints.at(k).y;
          rob.z = sutPoints.at(k).z;
          robot.at(2) = rob;

          if (reg2target(robot, world, r_2_w))
          {
            w_2_r = r_2_w.inv();
            outs.at(i) = outs.at(i) + w_2_r;
            ++count;
          }
        } // for (int k = j + 1; k < truth.size(); ++k)
      } // for (int j = 0; j < truth.size(); ++j)
      outs.at(i) = outs.at(i) / count;
    } // for (int i = 0; i < numRegs; ++i)

    return true;
  }

} // namespace Registration