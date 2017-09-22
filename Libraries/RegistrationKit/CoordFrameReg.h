///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Registration Toolkit
//  Workfile:        CoordFrameReg.h
//  Revision:        25 August, 2015
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Tools for registering between two coordinate systems.
///////////////////////////////////////////////////////////////////////////////

#ifndef REGISTRATION_KIT
#define REGISTRATION_KIT

#include "crpi.h"
#include "MatrixMath.h"
#include <iostream>
#include <vector>

using namespace std;
using namespace Math;

namespace Registration
{
  //! @brief Calculate the homogeneous transformation matrix from one coordinate frame (sut)
  //!        to another (tar)
  //!
  //! @param sutPoints Collection of points from the system under test's coordinate frame
  //! @param tarPoints Collection of corresponding points from the target coordinate frame
  //! @param sut_2_tar The 4x4 transformation from sut to tar
  //!
  //! @return True if operation completed successfully, False otherwise
  //!
  LIBRARY_API bool reg2target(vector<point> &sutPoints, vector<point> &tarPoints, matrix &out);

  //! @brief Calculate several local homogeneous transformation matrices from one coordinate frame
  //!        (sut) to another (tar) using unsupervised machine learning (clustering)
  //!
  //! @param sutPoints Collection of points from the system under test's coordinate frame
  //! @param tarPoints Collection of corresponding points from the target coordinate frame
  //! @param numRegs   The number of local registrations to generate
  //! @param kerkels   The spatial centers of the local registrations (in target coordinate space)
  //!                  corresponding with the registration matrices such that the closes one to a
  //!                  target location can be selected
  //! @param sut_2_tar The collection (of size numRegs) of 4x4 transformations from sut to tar
  //!
  //! @return True if the operation completed successfully, False otherwise
  //!
  LIBRARY_API bool reg2targetML(vector<point> &sutPoints,
                                vector<point> &tarPoints,
                                int numRegs,
                                vector<point> &kernels,
                                vector<matrix> &sut_2_tar);

} // namespace Registration

#endif