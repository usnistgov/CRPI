///////////////////////////////////////////////////////////////////////////////
//
//  Original System: CRPI
//  Subsystem:       Motion Capture Sensor
//  Workfile:        Vicon.h
//  Revision:        1.0 - 11 September, 2015
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Interface library for the Vicon motion capture system
//
///////////////////////////////////////////////////////////////////////////////

#ifndef VICON_H
#define VICON_H

#ifdef WIN32
#include <ulapi.h>
#include <crpi.h>
#include "MatrixMath.h"
#include "Client.h"
#else //linux
#include "../../ulapi/src/ulapi.h"
#include "../../CRPI/crpi.h"
#include "../../Math/MatrixMath.h"
#include "../../ThirdParty/Vicon/include/Client.h"
#endif

#include <vector>
#include <string.h>

#include "MoCapTypes.h"

using namespace std;
using namespace Math;

namespace Sensor
{
  //! @ingroup Sensor
  //!
  //! @brief   Interface class for the Vicon Tracker software system
  //!
  class LIBRARY_API Vicon
  {
  public:

    //! @brief Default constructor
    //!
    Vicon (char *ipAddress);

    //! @brief Default destructor
    //!
    ~Vicon ();

    //! @brief Query the motion capture tracking software for a list of identified rigid objects in the scene
    //!
    //! @param subjects Vector populated by the function with ViconSubject objects
    //!
    void GetCurrentSubjects(vector<MoCapSubject> &subjects);

    //! @brief Query the motion capture tracking software for a list of unlabeled markers found in the scene
    //!
    //! @param markers Vector populated by the function with point objects
    //!
    void GetUnlabeledMarkers(vector<point> &markers);
    
  private:

    //! @brief Handle for a separate thread that maintains contact with the Vicon system
    //!
    void *task_;

    //! @brief Data structure for communicating with a thread keeping the TCP/IP connection active
    //!
    keepalive ka_;

    //! @brief Temporary storage for Vicon tracked subject information
    //!
    MoCapSubject temp;

    //! @brief Temporary storage for a singular marker location
    //!
    point ptemp;

    //! @brief Vicon Tracker data stream client
    //!
    ViconDataStreamSDK::CPP::Client *client_;

  }; // Vicon
} // Sensor namespace

#endif
