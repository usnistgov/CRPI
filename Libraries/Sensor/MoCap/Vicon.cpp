///////////////////////////////////////////////////////////////////////////////
//
//  Original System: CRPI
//  Subsystem:       Motion Capture Sensor
//  Workfile:        Vicon.cpp
//  Revision:        1.0 - 11 September, 2015
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Simplified interface library for the Vicon MoCap system based on the
//  sample code provided by Vicon.
//
///////////////////////////////////////////////////////////////////////////////

#include "Vicon.h"

#include <iostream>
#include <fstream>
#include <cassert>


#ifdef WIN32
  #pragma warning (disable: 4204) 
  #include <conio.h>
  #include <cstdio>
  #include <ctime>
  #include <windows.h>
#else
  #include <time.h>
#endif

#include <time.h>

//#define VICON_NOISY

using namespace std;
using namespace ViconDataStreamSDK::CPP;

namespace Sensor
{
  namespace
  {
    std::string Adapt( const bool i_Value )
    {
      return i_Value ? "True" : "False";
    }

    std::string Adapt( const Direction::Enum i_Direction )
    {
      switch( i_Direction )
      {
        case Direction::Forward:
          return "Forward";
        case Direction::Backward:
          return "Backward";
        case Direction::Left:
          return "Left";
        case Direction::Right:
          return "Right";
        case Direction::Up:
          return "Up";
        case Direction::Down:
          return "Down";
        default:
          return "Unknown";
      }
    }

    std::string Adapt( const DeviceType::Enum i_DeviceType )
    {
      switch( i_DeviceType )
      {
        case DeviceType::ForcePlate:
          return "ForcePlate";
        case DeviceType::Unknown:
        default:
          return "Unknown";
      }
    }

    std::string Adapt( const Unit::Enum i_Unit )
    {
      switch( i_Unit )
      {
        case Unit::Meter:
          return "Meter";
        case Unit::Volt:
          return "Volt";
        case Unit::NewtonMeter:
          return "NewtonMeter";
        case Unit::Newton:
          return "Newton";
        case Unit::Kilogram:
          return "Kilogram";
        case Unit::Second:
          return "Second";
        case Unit::Ampere:
          return "Ampere";
        case Unit::Kelvin:
          return "Kelvin";
        case Unit::Mole:
          return "Mole";
        case Unit::Candela:
          return "Candela";
        case Unit::Radian:
          return "Radian";
        case Unit::Steradian:
          return "Steradian";
        case Unit::MeterSquared:
          return "MeterSquared";
        case Unit::MeterCubed:
          return "MeterCubed";
        case Unit::MeterPerSecond:
          return "MeterPerSecond";
        case Unit::MeterPerSecondSquared:
          return "MeterPerSecondSquared";
        case Unit::RadianPerSecond:
          return "RadianPerSecond";
        case Unit::RadianPerSecondSquared:
          return "RadianPerSecondSquared";
        case Unit::Hertz:
          return "Hertz";
        case Unit::Joule:
          return "Joule";
        case Unit::Watt:
          return "Watt";
        case Unit::Pascal:
          return "Pascal";
        case Unit::Lumen:
          return "Lumen";
        case Unit::Lux:
          return "Lux";
        case Unit::Coulomb:
          return "Coulomb";
        case Unit::Ohm:
          return "Ohm";
        case Unit::Farad:
          return "Farad";
        case Unit::Weber:
          return "Weber";
        case Unit::Tesla:
          return "Tesla";
        case Unit::Henry:
          return "Henry";
        case Unit::Siemens:
          return "Siemens";
        case Unit::Becquerel:
          return "Becquerel";
        case Unit::Gray:
          return "Gray";
        case Unit::Sievert:
          return "Sievert";
        case Unit::Katal:
          return "Katal";

        case Unit::Unknown:
        default:
          return "Unknown";
      }
    }
  }


  void livemanVicon (void *param)
  {
    keepalive *ka = (keepalive*)param;
    crpi_timer timer;

    size_t FrameRateWindow = 1000;
    size_t Counter = 0;
    clock_t LastTime = clock();
    Result::Enum gtfo;

    while (ka->runThread)
    {
      //! Grab the next frame
#ifdef VICON_NOISY
      cout << "Waiting for new frame...";
#endif
      do
      {
        ulapi_mutex_take(ka->handle);
        gtfo = ((Client*)ka->rob)->GetFrame().Result;
        ulapi_mutex_give(ka->handle);
        // Sleep a little so that we don't lumber the CPU with a busy poll
#ifdef WIN32
        Sleep (100);
#else
        usleep(100000);
#endif
#ifdef VICON_NOISY
        cout << ".";
#endif
      } while(gtfo != Result::Success );

#ifdef VICON_NOISY
      cout << endl;
#endif 
      timer.waitUntil(100);
    } // while (ka->runThread)

    return;
  }


  LIBRARY_API Vicon::Vicon (char * ipAddress)
  {
    task_ = ulapi_task_new();

    // Interface options  
    std::string HostName = ipAddress;
    HostName.append(":801");
    client_ = new Client();
    ka_.rob = client_;
    ka_.handle = ulapi_mutex_new(78);

    for(int i=0; i != 3; ++i) // repeat to check disconnecting doesn't wreck next connect
    {
      // Connect to a server
#ifdef VICON_NOISY
      cout << "Connecting to " << HostName << " ..." << std::flush;
#endif
      while(!((Client*)ka_.rob)->IsConnected().Connected)
      {
        // Direct connection
        bool ok = false;
        ok =(((Client*)ka_.rob)->Connect( HostName ).Result == Result::Success );
        if(!ok)
        {
          cout << "Warning - connect failed..." << endl;
        }
#ifdef VICON_NOISY
        cout << ".";
#endif
#ifdef WIN32
        Sleep (200);
#else
        usleep(200000);
#endif
      } // while( !((Client*)ka->rob)->IsConnected().Connected)
      cout << endl;
    } // 

    // Enable some different data types
    ((Client*)ka_.rob)->EnableSegmentData();
    ((Client*)ka_.rob)->EnableMarkerData();
    ((Client*)ka_.rob)->EnableUnlabeledMarkerData();
    ((Client*)ka_.rob)->EnableDeviceData();

#ifdef VICON_NOISY
    cout << "Segment Data Enabled: "          << Adapt( ((Client*)ka_.rob)->IsSegmentDataEnabled().Enabled )         << endl;
    cout << "Marker Data Enabled: "           << Adapt( ((Client*)ka_.rob)->IsMarkerDataEnabled().Enabled )          << endl;
    cout << "Unlabeled Marker Data Enabled: " << Adapt( ((Client*)ka_.rob)->IsUnlabeledMarkerDataEnabled().Enabled ) << endl;
    cout << "Device Data Enabled: "           << Adapt( ((Client*)ka_.rob)->IsDeviceDataEnabled().Enabled )          << endl;
#endif
    // Set the streaming mode
    //((Client*)ka_.rob)->SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPull );
    //((Client*)ka_.rob)->SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPullPreFetch );
    ((Client*)ka_.rob)->SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ServerPush );

    // Set the global up axis
    ((Client*)ka_.rob)->SetAxisMapping( Direction::Forward, 
                                        Direction::Left, 
                                        Direction::Up ); // Z-up

    Output_GetAxisMapping _Output_GetAxisMapping = ((Client*)ka_.rob)->GetAxisMapping();
#ifdef VICON_NOISY
    cout << "Axis Mapping: X-" << Adapt( _Output_GetAxisMapping.XAxis ) 
                           << " Y-" << Adapt( _Output_GetAxisMapping.YAxis ) 
                           << " Z-" << Adapt( _Output_GetAxisMapping.ZAxis ) << endl;
#endif

    ulapi_task_start((ulapi_task_struct*)task_, livemanVicon, &ka_, ulapi_prio_lowest(), 0);
  }


  LIBRARY_API Vicon::~Vicon ()
  {
    ((Client*)ka_.rob)->DisableSegmentData();
    ((Client*)ka_.rob)->DisableMarkerData();
    ((Client*)ka_.rob)->DisableUnlabeledMarkerData();
    ((Client*)ka_.rob)->DisableDeviceData();

#ifdef VICON_NOISY
    cout << " Disconnecting..." << endl;
#endif
    ((Client*)ka_.rob)->Disconnect();
  }


  LIBRARY_API void Vicon::GetCurrentSubjects(vector<MoCapSubject> &subjects)
  {
    subjects.clear();
    ulapi_mutex_take(ka_.handle);
    matrix rotation(3, 3);

    //! Count the number of subjects
    unsigned int SubjectCount = ((Client*)ka_.rob)->GetSubjectCount().SubjectCount;
#ifdef VICON_NOISY
    cout << "Subjects (" << SubjectCount << "):" << endl;
#endif

    for( unsigned int SubjectIndex = 0 ; SubjectIndex < SubjectCount ; ++SubjectIndex )
    {
#ifdef VICON_NOISY
      cout << "  Subject #" << SubjectIndex << endl;
#endif

      //! Get the subject name
      temp.name = ((Client*)ka_.rob)->GetSubjectName( SubjectIndex ).SubjectName;
#ifdef VICON_NOISY
      cout << "    Name: " << temp.name << endl;
#endif

      //! Count the number of segments
      unsigned int SegmentCount = ((Client*)ka_.rob)->GetSegmentCount( temp.name ).SegmentCount;
      for( unsigned int SegmentIndex = 0 ; SegmentIndex < SegmentCount ; ++SegmentIndex )
      {
        //! Get the segment name
        std::string SegmentName = ((Client*)ka_.rob)->GetSegmentName( temp.name, SegmentIndex ).SegmentName;

        //! Get the segment's children
        unsigned int ChildCount = ((Client*)ka_.rob)->GetSegmentChildCount( temp.name, SegmentName ).SegmentCount;
        for( unsigned int ChildIndex = 0 ; ChildIndex < ChildCount ; ++ChildIndex )
        {
          std::string ChildName = ((Client*)ka_.rob)->GetSegmentChildName( temp.name, SegmentName, ChildIndex ).SegmentName;
        }

        //! Get the global segment translation
        Output_GetSegmentGlobalTranslation _Output_GetSegmentGlobalTranslation = 
          ((Client*)ka_.rob)->GetSegmentGlobalTranslation( temp.name, SegmentName );
#ifdef VICON_NOISY
        cout << "        Global Translation: (" << _Output_GetSegmentGlobalTranslation.Translation[ 0 ]  << ", " 
              << _Output_GetSegmentGlobalTranslation.Translation[ 1 ]  << ", " 
              << _Output_GetSegmentGlobalTranslation.Translation[ 2 ]  << ") " 
              << Adapt( _Output_GetSegmentGlobalTranslation.Occluded ) << endl;
#endif
        temp.pose.x = _Output_GetSegmentGlobalTranslation.Translation[ 0 ];
        temp.pose.y = _Output_GetSegmentGlobalTranslation.Translation[ 1 ];
        temp.pose.z = _Output_GetSegmentGlobalTranslation.Translation[ 2 ];

        //! Get the global segment rotation as a matrix
        Output_GetSegmentGlobalRotationMatrix _Output_GetSegmentGlobalRotationMatrix = 
          ((Client*)ka_.rob)->GetSegmentGlobalRotationMatrix( temp.name, SegmentName );
#ifdef VICON_NOISY
        cout << "        Global Rotation Matrix: (" << _Output_GetSegmentGlobalRotationMatrix.Rotation[ 0 ]     << ", " 
              << _Output_GetSegmentGlobalRotationMatrix.Rotation[ 1 ]     << ", " 
              << _Output_GetSegmentGlobalRotationMatrix.Rotation[ 2 ]     << ", " 
              << _Output_GetSegmentGlobalRotationMatrix.Rotation[ 3 ]     << ", " 
              << _Output_GetSegmentGlobalRotationMatrix.Rotation[ 4 ]     << ", " 
              << _Output_GetSegmentGlobalRotationMatrix.Rotation[ 5 ]     << ", " 
              << _Output_GetSegmentGlobalRotationMatrix.Rotation[ 6 ]     << ", " 
              << _Output_GetSegmentGlobalRotationMatrix.Rotation[ 7 ]     << ", " 
              << _Output_GetSegmentGlobalRotationMatrix.Rotation[ 8 ]     << ") " 
              << Adapt( _Output_GetSegmentGlobalRotationMatrix.Occluded ) << endl;
#endif
        rotation.at(0, 0) = _Output_GetSegmentGlobalRotationMatrix.Rotation[ 0 ];
        rotation.at(0, 1) = _Output_GetSegmentGlobalRotationMatrix.Rotation[ 1 ];
        rotation.at(0, 2) = _Output_GetSegmentGlobalRotationMatrix.Rotation[ 2 ];
        rotation.at(1, 0) = _Output_GetSegmentGlobalRotationMatrix.Rotation[ 3 ];
        rotation.at(1, 1) = _Output_GetSegmentGlobalRotationMatrix.Rotation[ 4 ];
        rotation.at(1, 2) = _Output_GetSegmentGlobalRotationMatrix.Rotation[ 5 ];
        rotation.at(2, 0) = _Output_GetSegmentGlobalRotationMatrix.Rotation[ 6 ];
        rotation.at(2, 1) = _Output_GetSegmentGlobalRotationMatrix.Rotation[ 7 ];
        rotation.at(2, 2) = _Output_GetSegmentGlobalRotationMatrix.Rotation[ 8 ];

        //! Get the global segment rotation in EulerXYZ co-ordinates
        Output_GetSegmentGlobalRotationEulerXYZ _Output_GetSegmentGlobalRotationEulerXYZ = 
          ((Client*)ka_.rob)->GetSegmentGlobalRotationEulerXYZ( temp.name, SegmentName );
#ifdef VICON_NOISY
        cout << "        Global Rotation EulerXYZ: (" << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[ 0 ]     << ", " 
              << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[ 1 ]     << ", " 
              << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[ 2 ]     << ") " 
              << Adapt( _Output_GetSegmentGlobalRotationEulerXYZ.Occluded ) << endl;
#endif
        vector<double> vecout;
        rotation.rotMatrixEulerConvert (vecout);

        temp.pose.xr = vecout.at(0) * (180.0 / 3.141592654);
        temp.pose.yr = vecout.at(1) * (180.0 / 3.141592654);
        temp.pose.zr = vecout.at(2) * (180.0 / 3.141592654);
      } // for( unsigned int SegmentIndex = 0 ; SegmentIndex < SegmentCount ; ++SegmentIndex )

      temp.labeledMarkers.clear();

      //! Count the number of markers
      unsigned int MarkerCount = ((Client*)ka_.rob)->GetMarkerCount( temp.name ).MarkerCount;
#ifdef VICON_NOISY
      cout << "    Markers (" << MarkerCount << "):" << endl;
#endif

      for( unsigned int MarkerIndex = 0 ; MarkerIndex < MarkerCount ; ++MarkerIndex )
      {
        //! Get the marker name
        std::string MarkerName = ((Client*)ka_.rob)->GetMarkerName( temp.name, MarkerIndex ).MarkerName;

        //! Get the global marker translation
        Output_GetMarkerGlobalTranslation _Output_GetMarkerGlobalTranslation =
          ((Client*)ka_.rob)->GetMarkerGlobalTranslation( temp.name, MarkerName );
#ifdef VICON_NOISY
        cout << "      Marker #" << MarkerIndex            << ": "
              << MarkerName             << " ("
              << _Output_GetMarkerGlobalTranslation.Translation[ 0 ]  << ", "
              << _Output_GetMarkerGlobalTranslation.Translation[ 1 ]  << ", "
              << _Output_GetMarkerGlobalTranslation.Translation[ 2 ]  << ") "
              << Adapt( _Output_GetMarkerGlobalTranslation.Occluded ) << endl;
#endif
        ptemp.x = _Output_GetMarkerGlobalTranslation.Translation[ 0 ];
        ptemp.y = _Output_GetMarkerGlobalTranslation.Translation[ 1 ];
        ptemp.z = _Output_GetMarkerGlobalTranslation.Translation[ 2 ];
        temp.labeledMarkers.push_back(ptemp);
      } // for( unsigned int MarkerIndex = 0 ; MarkerIndex < MarkerCount ; ++MarkerIndex )
      temp.valid = true;

      subjects.push_back(temp);
    } // for( unsigned int SubjectIndex = 0 ; SubjectIndex < SubjectCount ; ++SubjectIndex )

    ulapi_mutex_give(ka_.handle);
  }


  LIBRARY_API void Vicon::GetUnlabeledMarkers(vector<point> &markers)
  {
    markers.clear();

    ulapi_mutex_take(ka_.handle);

      //! Get the unlabeled markers
    unsigned int UnlabeledMarkerCount = ((Client*)ka_.rob)->GetUnlabeledMarkerCount().MarkerCount;
#ifdef VICON_NOISY
    cout << "    Unlabeled Markers (" << UnlabeledMarkerCount << "):" << endl;
#endif
    for( unsigned int UnlabeledMarkerIndex = 0 ; UnlabeledMarkerIndex < UnlabeledMarkerCount ; ++UnlabeledMarkerIndex )
    { 
      // Get the global marker translation
      Output_GetUnlabeledMarkerGlobalTranslation _Output_GetUnlabeledMarkerGlobalTranslation =
        ((Client*)ka_.rob)->GetUnlabeledMarkerGlobalTranslation( UnlabeledMarkerIndex );
#ifdef VICON_NOISY
      cout << "      Marker #" << UnlabeledMarkerIndex   << ": ("
            << _Output_GetUnlabeledMarkerGlobalTranslation.Translation[ 0 ] << ", "
            << _Output_GetUnlabeledMarkerGlobalTranslation.Translation[ 1 ] << ", "
            << _Output_GetUnlabeledMarkerGlobalTranslation.Translation[ 2 ] << ")" << endl;
#endif
      ptemp.x = _Output_GetUnlabeledMarkerGlobalTranslation.Translation[ 0 ];
      ptemp.y = _Output_GetUnlabeledMarkerGlobalTranslation.Translation[ 0 ];
      ptemp.z = _Output_GetUnlabeledMarkerGlobalTranslation.Translation[ 0 ];
      markers.push_back(ptemp);
    } //for( unsigned int UnlabeledMarkerIndex = 0 ; UnlabeledMarkerIndex < UnlabeledMarkerCount ; ++UnlabeledMarkerIndex )
    ulapi_mutex_give(ka_.handle);
  }




}
