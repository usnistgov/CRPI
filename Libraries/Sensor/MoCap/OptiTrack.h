///////////////////////////////////////////////////////////////////////////////
//
//  Original System: CRPI
//  Subsystem:       Motion Capture Sensor
//  Workfile:        OptiTrack.h
//  Revision:        1.0 - 11 April, 2016
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Interface library for the OptiTrack Duo and Trio motion capture systems
//
//  To enable streaming from the Motive interface, select View->Data Streaming
//  Check the box labeled Broadcast Frame Data, and select all streams to be
//    broadcast.
//  Note that you may have to adjust the Local Interface option to get data
//    streaming to work properly for a localost connection.
//  Currently, this version only supports unicast.  Make sure the "Type" value
//    under the Advanced Network Settings is set to "Unicast."
//
///////////////////////////////////////////////////////////////////////////////

#ifndef OPTITRACK_H
#define OPTITRACK_H

#include <vector>
#include <string.h>

#if defined(_MSC_VER)
#include <ulapi.h>
#include <crpi.h>

#include "MatrixMath.h"

#include "NatNetTypes.h"
#include "NatNetClient.h"
#include "MoCapTypes.h"
#pragma warning( disable : 4996 )
using namespace Math;
#elif defined(__GNUC__)

#include "../../ulapi/src/ulapi.h"
#include "../../CRPI/crpi.h"

#include "../../Math/MatrixMath.h"

#include "../../ThirdParty/OptiTrack/include/NatNetTypes.h"
#include "../../ThirdParty/OptiTrack/include/NatNetClient.h"
#include "MoCapTypes.h"
#endif


using namespace std;


namespace Sensor
{
  //! @brief OptiTrack tracked object container
  //!
  typedef LIBRARY_API struct OTPointer_
  {
    //! @brief Collection of rigid bodies being tracked
    //!
    vector<MoCapSubject> subjects;

    //! @brief Collection of un-named markers detected in the scene
    //!
    vector<point>        markers;

    //! @brief Handle of the OptiTrack sensor instance
    //!
    NatNetClient         *client;

    //! @brief Mutex handle for data protection
    //!
    ulapi_mutex_struct   *handle;

    //! @brief Flag to specify when threads should be killed cleanly and remotely
    //!
    bool                 runThread;

    //! @brief Default constructor
    //!
    OTPointer_()
    {
      runThread = true;
    }

    //! @brief Default destructor
    //!
    ~OTPointer_()
    {
      markers.clear();
    }
  } OTPointer;


  //! @ingroup Sensor
  //!
  //! @brief   Interface class for the OptiTrack motion capture system
  //!
  class LIBRARY_API OptiTrack
  {
  public:

    //! @brief Default constructor
    //!
    OptiTrack(char *ipAddress);

    //! @brief Default destructor
    //!
    ~OptiTrack();

    //! @brief Query the motion capture tracking software for a list of identified rigid objects in the scene
    //!
    //! @param subjects Vector populated by the function with OptiTrackSubject objects
    //!
    void GetCurrentSubjects(vector<MoCapSubject> &subjects);

    //! @brief Query the motion capture tracking software for a list of unlabeled markers found in the scene
    //!
    //! @param markers Vector populated by the function with point objects
    //!
    void GetUnlabeledMarkers(vector<point> &markers);
    
  private:

    //! @brief Handle for a separate thread that maintains contact with the OptiTrack system
    //!
    void *task_;

    //! @brief OptiTrack motion capture data handle
    //!
    OTPointer *otp_;

    //! @brief Temporary storage for a singular marker location
    //!
    point ptemp;

    //! @brief OptiTrack sensor handle
    //!
    NatNetClient* Client_;
  }; // OptiTrack
} // Sensor namespace

#endif
