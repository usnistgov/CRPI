///////////////////////////////////////////////////////////////////////////////
//
//  Original System: CRPI
//  Subsystem:       Motion Capture Sensor
//  Workfile:        OptiTrack.cpp
//  Revision:        1.0 - 11 September, 2015
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Simplified interface library for the OptiTrack MoCap system
//
///////////////////////////////////////////////////////////////////////////////

#include "OptiTrack.h"

#include <iostream>
#include <fstream>
#include <cassert>
#include <ctime>

#ifdef WIN32
  #pragma warning (disable: 4204) 
  #include <conio.h>   // For _kbhit()
  #include <cstdio>    // For getchar()
  #include <windows.h> // For Sleep()
#else //linux
  #define __cdecl __attribute__((__cdecl__))
#endif // WIN32

#include <time.h>

//#define OPTITRACK_NOISY

using namespace std;

namespace Sensor
{
  void __cdecl MsgHandler(int msgType, char* msg)
  {
    printf("\n%s\n", msg);
  }

  void __cdecl DataHandler(sFrameOfMocapData* data, void* pUserData)
  {
    OTPointer *otp = (OTPointer*)pUserData;
    int i = 0;
    Math::point pt;
    char name[128];
    vector<double> q;
    vector<double> e;

#ifdef OPTITRACK_NOISY
    printf("FrameID : %d\n", data->iFrame);
    printf("Timestamp :  %3.2lf\n", data->fTimestamp);
    printf("Latency :  %3.2lf\n", data->fLatency);

    bool bIsRecording = ((data->params & 0x01) != 0);
    bool bTrackedModelsChanged = ((data->params & 0x02) != 0);
    if (bIsRecording)
    {
      printf("RECORDING\n");
    }
    if (bTrackedModelsChanged)
    {
      printf("Models Changed.\n");
    }
#endif

    //! Timecode - for systems with an eSync and SMPTE timecode generator - decode to values
    int hour, minute, second, frame, subframe;
    bool bValid = otp->client->DecodeTimecode(data->Timecode, data->TimecodeSubframe, &hour, &minute, &second, &frame, &subframe);

    //! Decode to friendly string
#ifdef OPTITRACK_NOISY
    char szTimecode[128] = "";
    otp->client->TimecodeStringify(data->Timecode, data->TimecodeSubframe, szTimecode, 128);
    printf("Timecode : %s\n", szTimecode);
#endif

    //! Other Markers (unlabeled)
#ifdef OPTITRACK_NOISY
    printf("Other Markers [Count=%d]\n", data->nOtherMarkers);
#endif

    ulapi_mutex_take(otp->handle);

    otp->markers.clear();
    for (i = 0; i < data->nOtherMarkers; ++i)
    {
      pt.x = data->OtherMarkers[i][0];
      pt.y = data->OtherMarkers[i][1];
      pt.z = data->OtherMarkers[i][2];
#ifdef OPTITRACK_NOISY
      printf("Other Marker %d : %3.2f\t%3.2f\t%3.2f\n", i, pt.x, pt.y, pt.z);
#endif
      otp->markers.push_back(pt);
    } // for (i = 0; i < data->nOtherMarkers; ++i)

    //! Rigid Bodies
#ifdef OPTITRACK_NOISY
    printf("Rigid Bodies [Count=%d]\n", data->nRigidBodies);
#endif

    otp->subjects.clear();
    for (i = 0; i < data->nRigidBodies; ++i)
    {
      //! 0x01 : bool, rigid body was successfully tracked in this frame
      bool bTrackingValid = data->RigidBodies[i].params & 0x01;

      if (bTrackingValid)
      {
        MoCapSubject sub;
        //subject = new OptiTrackSubject();

        q.clear();
        q.push_back(data->RigidBodies[i].qw);
        q.push_back(data->RigidBodies[i].qx);
        q.push_back(data->RigidBodies[i].qy);
        q.push_back(data->RigidBodies[i].qz);

        bool val = sub.rotation.rotQuaternionMatrixConvert(q);
        val = sub.rotation.rotMatrixEulerConvert(e);
        sub.pose.xr = e.at(0);
        sub.pose.yr = e.at(1);
        sub.pose.zr = e.at(2);

        sub.pose.x = data->RigidBodies[i].x;
        sub.pose.y = data->RigidBodies[i].y;
        sub.pose.z = data->RigidBodies[i].z;

        sprintf(name, "%d", data->RigidBodies[i].ID);
        sub.name = name;

#ifdef OPTITRACK_NOISY
        printf("Rigid Body [ID=%s  Error=%3.2f  Valid=%d]\n", sub.name.c_str(), data->RigidBodies[i].MeanError, bTrackingValid);
        printf("\tx\ty\tz\trx\try\trz\n");
        printf("\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n", sub.pose.x, sub.pose.y, sub.pose.z, sub.pose.xrot, sub.pose.yrot, sub.pose.zrot);
#endif

#ifdef OPTITRACK_NOISY
        printf("\tRigid body markers [Count=%d]\n", data->RigidBodies[i].nMarkers);
#endif
        sub.labeledMarkers.clear();
        for (int iMarker = 0; iMarker < data->RigidBodies[i].nMarkers; ++iMarker)
        {
          if (data->RigidBodies[i].Markers)
          {
            pt.x = data->RigidBodies[i].Markers[iMarker][0];
            pt.y = data->RigidBodies[i].Markers[iMarker][1];
            pt.z = data->RigidBodies[i].Markers[iMarker][2];
          }
          sub.labeledMarkers.push_back(pt);
#ifdef OPTITRACK_NOISY
          printf("\t\t");
          if (data->RigidBodies[i].MarkerIDs)
          {
            printf("MarkerID:%d", data->RigidBodies[i].MarkerIDs[iMarker]);
          }
          if (data->RigidBodies[i].MarkerSizes)
          {
            printf("\tMarkerSize:%3.2f", data->RigidBodies[i].MarkerSizes[iMarker]);
          }
          if (data->RigidBodies[i].Markers)
          {
            printf("\tMarkerPos:%3.2f,%3.2f,%3.2f\n", pt.x, pt.y, pt.z);
          }
#endif
        } // for (int iMarker = 0; iMarker < data->RigidBodies[i].nMarkers; ++iMarker)
        sub.valid = true;
        otp->subjects.push_back(sub);
      } // if (bTrackingValid)
    } // for (i = 0; i < data->nRigidBodies; ++i)
    ulapi_mutex_give(otp->handle);
  }


  LIBRARY_API OptiTrack::OptiTrack (char * ipAddress)
  {
    task_ = ulapi_task_new();
    otp_ = new OTPointer;

    // Interface options  
    std::string HostName = ipAddress;
    otp_->handle = ulapi_mutex_new(78);

    int result;
    Client_ = new NatNetClient(ConnectionType_Unicast);
    otp_->client = Client_;

    Client_->SetVerbosityLevel(Verbosity_Warning);
    Client_->SetMessageCallback(MsgHandler);
    Client_->SetDataCallback(DataHandler, otp_);
    
    //! Get version information
    unsigned char ver[4];
    Client_->NatNetVersion(ver);

    char myIPAddress[128];
    strcpy(myIPAddress, "");

    bool okay = false;
    
    do
    {
      //! Initialize client
      result = Client_->Initialize(myIPAddress, ipAddress);
      if (result != ErrorCode_OK)
      {
        printf("Could not connect to OptiTrack server.");
      }
      else
      {
        //! Get server information
        sServerDescription sd;
        memset(&sd, 0, sizeof(sd));
        Client_->GetServerDescription(&sd);
        if (!sd.HostPresent)
        {
          printf("Unable to connect to OptiTrack server.  Host not present.");
        }
        else
        {
          //! We are connected and ready to go
          okay = true;
        }
      }
    } while (okay == false);
  }


  LIBRARY_API OptiTrack::~OptiTrack ()
  {
    Client_->Uninitialize();
  }


  LIBRARY_API void OptiTrack::GetCurrentSubjects(vector<MoCapSubject> &subjects)
  {
    subjects.clear();

    ulapi_mutex_take(otp_->handle);
    subjects = otp_->subjects;
    ulapi_mutex_give(otp_->handle);
  }


  LIBRARY_API void OptiTrack::GetUnlabeledMarkers(vector<point> &markers)
  {
    markers.clear();

    ulapi_mutex_take(otp_->handle);
    markers = otp_->markers;
    ulapi_mutex_give(otp_->handle);
  }
}
