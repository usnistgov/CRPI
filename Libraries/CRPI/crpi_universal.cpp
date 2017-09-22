///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Robot Interface
//  Workfile:        crpi_universal.cpp
//  Revision:        1.0 - 24 June, 2014
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Universal Robot UR10 interface definitions.
//`
///////////////////////////////////////////////////////////////////////////////

#include "crpi_universal.h"
#include <fstream>
#include <iostream>

#define BLOCKING_MOTION
//#define VERIFY_MOVING
#define USE_TIMEOUT

#define NEWTCPIP

//#define UNIVERSAL_NOISY
#define distthresh 0.005f
#define angthresh 0.05f
#define timethresh 10

using namespace std;

namespace crpi_robot
{
  void livemanUniversal(void *param)
  {
    universalHandler *uh = (universalHandler*)param;
    robotPose pose;
    crpi_timer timer;
    ulapi_integer get;
    char buffer[4];

    while (uh->runThread)
    {
      ulapi_mutex_take(uh->TCPIPhandle);
      get = ulapi_socket_read(uh->clientID, buffer, 4);
      ulapi_mutex_give(uh->TCPIPhandle);

      //! Don't slam your processor!  You don't need to poll at full speed.
      timer.waitUntil(5000);
    }
    return;
  }


  int readInt (char *buffer, int &index, bool little)
  {
    static char ucval[sizeof(int)];
    static int x;
    static int returnMe;

    for (x = 0; x < sizeof(int); ++x)
    {
      ucval[x] = (little ? buffer[index+(sizeof(int) - (x+1))] : buffer[index + x]);
    }
    index += sizeof(int);
    memcpy(&returnMe, ucval, sizeof(int));
    return returnMe;
  }


  double readDouble (char *buffer, int &index, bool little)
  {
    static char ucval[sizeof(double)];
    static int x;
    static double returnMe;

    for (x = 0; x < sizeof(double); ++x)
    {
      ucval[x] = (little ? buffer[index+(sizeof(double) - (x+1))] : buffer[index + x]);
    }
    index += sizeof(double);
    memcpy(&returnMe, ucval, sizeof(double));
    return returnMe;
  }


  long readLong (char *buffer, int &index, bool little)
  {
    static char ucval[sizeof(double)];
    static int x;
    static long returnMe;

    for (x = 0; x < sizeof(long); ++x)
    {
      ucval[x] = (little ? buffer[index+(sizeof(long) - (x+1))] : buffer[index + x]);
    }
    index += sizeof(long);
    memcpy(&returnMe, ucval, sizeof(long));
    return returnMe;
  }

  bool parseFeedback (int bytes, char *buffer, robotPose &pose, robotAxes &axes, robotIO &io, robotPose &forces, robotPose &speeds)
  {
    double dval;
    int ival;
    bool little = false;

    //! JAM:  Note that the UR sends feedback in big endian format.  Test for endianess.
    int i = 0x01234567;
    int j;
    char* c = (char*)&i;
    little = (c[0] == 'g');
    //for (j = 0; j < sizeof(int); j++)
    //{
    //  printf(" %.2x", c[j]);
    //}
    //printf("\n");
       
    //! If big endian:  01 23 45 67
    //! If little endian: 67 45 23 01
    
    //cout << bytes << endl;
    if (bytes != 1044 && bytes != 812)
    {
      //! unknown byte length
      return false;
    }

    int index = 0;
    //! Number of bytes sent
    ival = readInt(buffer, index, little);

    //! Time
    dval = readDouble(buffer, index, little);

    //! Target joint positions
    for (j = 0; j < 6; ++j)
    {
      //dval = readDouble(buffer, index, little);
      axes.axis.at(j) = readDouble(buffer, index, little);
    }

    //! Target joint velocities
    for (j = 0; j < 6; ++j)
    {
      dval = readDouble(buffer, index, little);
    }

    //! Target joint accelerations
    for (j = 0; j < 6; ++j)
    {
      dval = readDouble(buffer, index, little);
    }

    //! Target joint current
    for (j = 0; j < 6; ++j)
    {
      dval = readDouble(buffer, index, little);
    }

    //! Target joint moments
    for (j = 0; j < 6; ++j)
    {
      dval = readDouble(buffer, index, little);
    }

    //! Actual joint positions
    for (j = 0; j < 6; ++j)
    {
      dval = readDouble(buffer, index, little);
    }

    //! Actual joint velocities
    for (j = 0; j < 6; ++j)
    {
      dval = readDouble(buffer, index, little);
    }

    //! Actual joint currents
    for (j = 0; j < 6; ++j)
    {
      dval = readDouble(buffer, index, little);
    }

    //! Joint control currents
    for (j = 0; j < 6; ++j)
    {
      dval = readDouble(buffer, index, little);
    }

    //! Actual Cartesian coordinates of TCP
    for (j = 0; j < 6; ++j)
    {
      dval = readDouble(buffer, index, little);
    }

    //! Actual speed of TCP in Cartesian space
    for (j = 0; j < 6; ++j)
    {
      dval = readDouble(buffer, index, little);
    }

    //! Forces at TCP
    /*
    for (j = 0; j < 6; ++j)
    {
      dval = readDouble(buffer, index, little);
    }
    */
    forces.x = readDouble(buffer, index, little);
    forces.y = readDouble(buffer, index, little);
    forces.z = readDouble(buffer, index, little);
    forces.xrot = readDouble(buffer, index, little);
    forces.yrot = readDouble(buffer, index, little);
    forces.zrot = readDouble(buffer, index, little);

    //! TCP target coordinates
    /*
    for (j = 0; j < 6; ++j)
    {
      dval = readDouble(buffer, index, little);
    }
    */
    pose.x = readDouble(buffer, index, little);
    pose.y = readDouble(buffer, index, little);
    pose.z = readDouble(buffer, index, little);
    pose.xrot = readDouble(buffer, index, little);
    pose.yrot = readDouble(buffer, index, little);
    pose.zrot = readDouble(buffer, index, little);
    //cout << "raw: " << pose.x << " " << pose.y << " " << pose.z << " " << pose.xrot << " " << pose.yrot << " " << pose.zrot << endl;

    //! TCP target speed
    /*
    for (j = 0; j < 6; ++j)
    {
      dval = readDouble(buffer, index, little);
    }
    */
    speeds.x = readDouble(buffer, index, little);
    speeds.y = readDouble(buffer, index, little);
    speeds.z = readDouble(buffer, index, little);
    speeds.xrot = readDouble(buffer, index, little);
    speeds.yrot = readDouble(buffer, index, little);
    speeds.zrot = readDouble(buffer, index, little);
    
    //! Digital input states
    ival = (int)(readDouble(buffer, index, little));
    for (j = (CRPI_IO_MAX-1); j >= 0; j--)
    {
      io.dio[j] = (ival >= (1 << j));
      if (io.dio[j])
      {
        ival -= (int)(1 << j);
      }
    }

    //! Motor temperatures
    for (j = 0; j < 6; ++j)
    {
      dval = readDouble(buffer, index, little);
    }

    //! Controller timer
    dval = readDouble(buffer, index, little);

    //! Test value
    dval = readDouble(buffer, index, little);

    //! Robot mode
    dval = readDouble(buffer, index, little);

    //! Joint modes
    for (j = 0; j < 6; ++j)
    {
      dval = readDouble(buffer, index, little);
    }

    //! 1044 bytes beyond this point
    if (ival > 812)
    {
      //! Safety mode
      dval = readDouble(buffer, index, little);

      //! Reserved
      for (j = 0; j < 6; ++j)
      {
        dval = readDouble(buffer, index, little);
      }

      //! Tool accelerometer
      for (j = 0; j < 3; ++j)
      {
        dval = readDouble(buffer, index, little);
      }

      //! Reserved
      for (j = 0; j < 6; ++j)
      {
        dval = readDouble(buffer, index, little);
      }

      //! Speed scaling
      dval = readDouble(buffer, index, little);

      //! Linear momentum norm
      dval = readDouble(buffer, index, little);

      //! Reserved
      dval = readDouble(buffer, index, little);

      //! Reserved
      dval = readDouble(buffer, index, little);

      //! Main voltage
      dval = readDouble(buffer, index, little);

      //! Robot voltage
      dval = readDouble(buffer, index, little);

      //! Robot current
      dval = readDouble(buffer, index, little);

      //! Joint voltages
      for (j = 0; j < 6; ++j)
      {
        dval = readDouble(buffer, index, little);
      }
    } // if (ival > 812)

    return true;
  }

  void feedbackThread (void *param)
  {
    static crpi_timer timer;
    universalHandler *uH = (universalHandler*)param;
    bool connected = false;
    char *buffer;
    int get;
    robotPose pose;
    robotAxes axes;
    robotPose force;
    robotPose speed;
    robotIO io;

    buffer = new char[1044];

    while (uH->runThread)
    {
//      cout << "running... " << endl;
      /*
      HOST = "169.254.152.50" //! The remote host
      PORT = 30003            //! 125 Hz update of robot state
      PORT = 30002            //! Control port
      */
      ulapi_integer client = ulapi_socket_get_client_id (30003, uH->params.tcp_ip_addr);
      ulapi_socket_set_nonblocking(client);

      if (client > 0)
      {
        //! Read feedback from robot
        get = ulapi_socket_read(client, buffer, 1044);
        ulapi_socket_close(client);

        //! Parse feedback from robot
        if (parseFeedback(get, buffer, pose, axes, io, force, speed))
        {
          ulapi_mutex_take(uH->handle);
          //! Store feedback from robot
          uH->curPose = pose;
          uH->poseGood = true;
          uH->curAxes = axes;
          uH->curForces = force;
          uH->curSpeeds = speed;
          uH->curIO = io;
          ulapi_mutex_give(uH->handle);

          //cout << "(" << pose.x << ", " << pose.y << ", " << pose.z << ", " << pose.xrot << ", " << pose.yrot << ", " << pose.zrot << ")" << endl;
          //cout << "(" << axes.axis.at(0) << ", " << axes.axis.at(1) << ", " << axes.axis.at(2) << ", " << axes.axis.at(3) << ", " << axes.axis.at(4) << ", " << axes.axis.at(5) << ")" << endl;
        }
      }

      //! Don't slam your processor!  You don't need to poll at full speed. 10 Hz
      //timer.waitUntil(100);//Sleep (100);
      Sleep(50);
    }
    delete [] buffer;
    return;
  }


  LIBRARY_API CrpiUniversal::CrpiUniversal (CrpiRobotParams &params) :
    firstIO_(true)
  {
    double Xtheta, Ytheta, Ztheta;
    params_ = params;
    handle_.params = params_;
    
    maxSpeed_ = 1.0f;
    maxAccel_ = 3.0f;

    //! These are nominal values defined by trial and error.  Use the SetAbsoluteSpeed and
    //! SetAbsoluteAcceleration commands to override
    speed_ = 1.0f;
    acceleration_ = 0.2f;

    mssgBuffer_ = new char[8192];
    ulapi_init();

    angleUnits_ = RADIAN;
    lengthUnits_ = METER;
    for (int i = 0; i < 6; ++i)
    {
      axialUnits_[i] = RADIAN;
    }

    task = ulapi_task_new();
    handle_.handle = ulapi_mutex_new(19);
    handle_.TCPIPhandle = ulapi_mutex_new(17);
    handle_.rob = this;
    handle_.runThread = true;
    handle_.poseGood = false;
    handle_.curTool = -1;

    //! Connect to UR server
#ifdef NEWTCPIP
    handle_.clientID = ulapi_socket_get_client_id(params_.tcp_ip_port, params_.tcp_ip_addr);
    ulapi_socket_set_nonblocking(handle_.clientID);
#endif
    ulapi_task_start((ulapi_task_struct*)task, feedbackThread, &handle_, ulapi_prio_lowest(), 0);

    while (handle_.poseGood != true)
    {
      Sleep(100);
    }

#ifdef NEWTCPIP
    ulapi_task_start((ulapi_task_struct*)task, livemanUniversal, &handle_, ulapi_prio_lowest(), 0);
#endif

    pin_ = new matrix(3,1);
    pout_ = new matrix(3,1);

    forward_ = new matrix(4, 4);
    backward_ = new matrix(4, 4);
    matrix r(3, 3);

    Xtheta = params_.mounting->xrot * (3.141592654 / 180.0f);
    Ytheta = params_.mounting->yrot * (3.141592654 / 180.0f);
    Ztheta = params_.mounting->zrot * (3.141592654 / 180.0f);
        
    vector<double> vtemp;
    vtemp.push_back(Xtheta);
    vtemp.push_back(Ytheta);
    vtemp.push_back(Ztheta);

    r.rotEulerMatrixConvert(vtemp);
    //! Copy rotation matrix to homogeneous transformation matrix
    for (int x = 0; x < 3; ++x)
    {
      for (int y = 0; y < 3; ++y)
      {
        forward_->at(x, y) = r.at(x, y);
      }
    }
    forward_->at(0, 3) = params_.mounting->x;
    forward_->at(1, 3) = params_.mounting->y;
    forward_->at(2, 3) = params_.mounting->z;
    forward_->at(3, 3) = 1.0f;
    *backward_ = forward_->inv(); //JAM forward_->matrixInv(*forward_, *backward_); // 
  }


  LIBRARY_API CrpiUniversal::~CrpiUniversal ()
  {
    handle_.runThread = false;
    delete forward_;
    delete backward_;
    delete pin_;
    delete pout_;
  }

  LIBRARY_API CanonReturn CrpiUniversal::ApplyCartesianForceTorque (robotPose &robotForceTorque, vector<bool> activeAxes, vector<bool> manipulator)
  {
    //! TODO
    return CANON_FAILURE;
  }

  LIBRARY_API CanonReturn CrpiUniversal::ApplyJointTorque (robotAxes &robotJointTorque)
  {
    //! TODO
    return CANON_FAILURE;
  }

  LIBRARY_API CanonReturn CrpiUniversal::SetTool (double percent)
  {
    //! There is no automatic handler on the UR side for generic tool actuation.  Use the digital output
    //! command for binary controller-based actuation (e.g., pneumatic parallel).
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiUniversal::Couple (const char *targetID)
  {
    std::vector<CrpiToolDef>::const_iterator itr;
    int tool = 0;
    for (itr = params_.tools.begin(); itr != params_.tools.end(); ++itr, ++tool)
    {
      if (strcmp(targetID, itr->toolName.c_str()) == 0)
      {
        break;
      }
    }
    if (itr == params_.tools.end())
    {
      handle_.curTool = -1;
      return CANON_FAILURE;
    }
    handle_.curTool = tool;
    vector<double> target;

    //! TODO:  Fix this to use the correct units instead of assuming values in mm
    target.push_back (itr->TCP.x / 1000.0f);
    target.push_back (itr->TCP.y / 1000.0f);
    target.push_back (itr->TCP.z / 1000.0f);
    target.push_back (itr->TCP.xrot);
    target.push_back (itr->TCP.yrot);
    target.push_back (itr->TCP.zrot);


    //! Tool definition found
    if (generateParameter('T', 'D', target))
    {
      //cout << "sending..." << endl;
      //cout << handle_.moveMe.str().c_str();
      if (send())
      {
        //! Okay
      }
      else
      {
        //! Oops
        return CANON_FAILURE;
      }
    }

    target.clear();
    target.push_back (itr->mass);
    target.push_back (itr->centerMass.x / 1000.0f);
    target.push_back (itr->centerMass.y / 1000.0f);
    target.push_back (itr->centerMass.z / 1000.0f);
    
    //! Tool definition found
    if (generateParameter('T', 'C', target))
    {
      if (send())
      {
        //! Okay
        
      }
      else
      {
        //! Oops
        return CANON_FAILURE;
      }
    }


    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiUniversal::Message (const char *message)
  {
    ulapi_mutex_take(handle_.handle);
    handle_.moveMe.str(string());
    handle_.moveMe << "def myProg():\n";
    handle_.moveMe << "popup(\"" << message << "\", title='CRPI Message', warning=False, error=False)\n";
    handle_.moveMe << "end\n";
    ulapi_mutex_give(handle_.handle);

    //! Send message to robot
    if (!send())
    {
      //! error sending
      return CANON_FAILURE;
    }
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiUniversal::MoveStraightTo (robotPose &pose)
  {
    //! Construct message
    vector<double> target;
    robotPose temp;

#ifdef BLOCKING_MOTION 
    double dist, dist2, tim, dist_rot;
    int count;
#endif

    transformToMount(pose, temp);
    target.push_back (temp.x);
    target.push_back (temp.y);
    target.push_back (temp.z);
    target.push_back (temp.xrot);
    target.push_back (temp.yrot);
    target.push_back (temp.zrot);

    //! LIN, Cartesian, Absolute
    if (generateMove ('L', 'C', 'A', target))
    {
      //! Send message to robot
      if (!send())
      {
        //! error sending
        return CANON_FAILURE;
      }
#ifdef BLOCKING_MOTION     
      //! ROBOT DOES NOT BLOCK:  WAIT FOR RESPONSE
      dist2 = 1000.0;
      count = 0;
      tim = ulapi_time();
      while (true)
      {
        ulapi_mutex_take(handle_.handle);
        dist = handle_.curPose.distance(temp);
        dist_rot = handle_.curPose.distance_rot(temp);
        ulapi_mutex_give(handle_.handle);

#ifdef VERIFY_MOVING
        if (dist >= dist2)
        {
          ++count;

          if (count >= 30)
          {
            //! Robot is not moving.  Retry.
            return CANON_FAILURE;
          }
        }
#endif

#ifdef USE_TIMEOUT
        if ((ulapi_time() - tim) > timethresh)
        {
          return CANON_FAILURE;
        }
#endif
        //cout << "distance rot: " << dist_rot << endl;
        //printf("d %f a %f\n", dist, dist_rot);
        if (dist <= distthresh && dist_rot <= angthresh)
        {
          break;
        }
        Sleep(100);
//        ulapi_wait(200000);
        dist2 = dist;
      }
#endif
    }
    else
    {
      //! Error generating motion message
      return CANON_FAILURE;
    }

    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiUniversal::MoveThroughTo (robotPose *poses,
                                                        int numPoses,
                                                        robotPose *accelerations,
                                                        robotPose *speeds,
                                                        robotPose *tolerances)
  {
    bool status = true;

    //! This is a temporary function definition until the KRL code has been updated to properly handle
    //! this method
    for (int x = 0; x < numPoses; ++x)
    {
      status &= (MoveTo (poses[x]) == CANON_SUCCESS);
      if (!status)
      {
        //! Error when executing multi move
        return CANON_FAILURE;
      }
    }

    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiUniversal::MoveTo (robotPose &pose)
  {
    //! Construct message
    vector<double> target;
    robotPose temp = pose;

#ifdef BLOCKING_MOTION 
    double dist, dist2, tim;
    int count;
#endif

    transformToMount(pose, temp);
    target.push_back (temp.x);
    target.push_back (temp.y);
    target.push_back (temp.z);
    target.push_back (temp.xrot);
    target.push_back (temp.yrot);
    target.push_back (temp.zrot);

/*
    if (lengthUnits_ == MM)
    {
      temp.x *= 0.001f;
      temp.y *= 0.001f;
      temp.z *= 0.001f;
    }
    else if (lengthUnits_ == INCH)
    {
      //! TODO: CONVERT TO METERS
    }

    if (angleUnits_ == DEGREE)
    {
      temp.xrot *= (3.141592654f / 180.0f);
      temp.yrot *= (3.141592654f / 180.0f);
      temp.zrot *= (3.141592654f / 180.0f);
    }

    pin_->at(0,0) = temp.x;
    pin_->at(1,0) = temp.y;
    pin_->at(2,0) = temp.z;
    pin_->matrixMult (*backward_, *pin_, *pout_);
    temp.x = pout_->at(0,0);
    temp.y = pout_->at(1,0);
    temp.z = pout_->at(2,0);

    target.push_back (temp.x);
    target.push_back (temp.y);
    target.push_back (temp.z);
    target.push_back (temp.xrot);
    target.push_back (temp.yrot);
    target.push_back (temp.zrot);
*/
    //! PTP, Cartesian, Absolute
    //if (generateMove ('P', 'C', 'R', target))
    if (generateMove ('P', 'C', 'A', target))
    {
      //! Send message to robot
      if (!send())
      {
        //! error sending
        return CANON_FAILURE;
      }
#ifdef BLOCKING_MOTION     
      //! ROBOT DOES NOT BLOCK:  WAIT FOR RESPONSE
      dist2 = 1000.0;
      count = 0;
      tim = ulapi_time();
      while (true)
      {
        ulapi_mutex_take(handle_.handle);
        dist = handle_.curPose.distance(temp);
        ulapi_mutex_give(handle_.handle);

#ifdef VERIFY_MOVING
        if (dist >= dist2)
        {
          ++count;

          if (count >= 30)
          {
            //! Robot is not moving.  Retry.
            return CANON_FAILURE;
          }
        }
#endif

#ifdef USE_TIMEOUT
        if ((ulapi_time() - tim) > timethresh)
        {
          return CANON_FAILURE;
        }
#endif
        //printf("d %f\n", dist);
        if (dist <= distthresh)
        {
          break;
        }
        Sleep(100);
//        ulapi_wait(200000);
        dist2 = dist;
      }
#endif

    }
    else
    {
      //! Error generating motion message
      return CANON_FAILURE;
    }

    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiUniversal::GetRobotAxes (robotAxes *axes)
  {
    ulapi_mutex_take(handle_.handle);
    *axes = handle_.curAxes;

    for (int i = 0; i < 6; ++i)
    {
//      if (axialUnits_[i] == DEGREE)
      {
        axes->axis.at(i) *= (180.0f / 3.141592654f);
      }
    }

    ulapi_mutex_give(handle_.handle);

    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiUniversal::GetRobotForces (robotPose *forces)
  {
    robotPose temp;
    matrix pintemp(4,4), r(3,3);
    matrix pouttemp(4, 4);

    ulapi_mutex_take(handle_.handle);
    *forces = handle_.curForces;
    ulapi_mutex_give(handle_.handle);
    
    transformFromMount(handle_.curForces, temp, false);

    forces->x = temp.x;
    forces->y = temp.y;
    forces->z = temp.z;
    forces->xrot = temp.xrot;
    forces->yrot = temp.yrot;
    forces->zrot = temp.zrot;

    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiUniversal::GetRobotIO (robotIO *io)
  {
    ulapi_mutex_take(handle_.handle);
    *io = handle_.curIO;
    ulapi_mutex_give(handle_.handle);
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiUniversal::GetRobotPose (robotPose *pose)
  {
    robotPose temp;
    matrix pintemp(4,4), r(3,3), rtmp1(3,3);
    matrix pouttemp(4, 4);

    ulapi_mutex_take(handle_.handle);
    *pose = handle_.curPose;
    ulapi_mutex_give(handle_.handle);

#ifdef UNIVERSAL_NOISY
    cout << "raw: (" << handle_.curPose.x << ", " << handle_.curPose.y << ", " << handle_.curPose.z << ", " << handle_.curPose.xrot << ", " << handle_.curPose.yrot << ", " << handle_.curPose.zrot << ")" << endl;
#endif
    
    transformFromMount(handle_.curPose, temp);
  
    pose->x = temp.x;
    pose->y = temp.y;
    pose->z = temp.z;
    pose->xrot = temp.xrot;
    pose->yrot = temp.yrot;
    pose->zrot = temp.zrot;

    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiUniversal::GetRobotSpeed (robotPose *speed)
  {    
    robotPose temp;
    matrix pintemp(4,4), r(3,3);
    matrix pouttemp(4, 4);
    
    ulapi_mutex_take(handle_.handle);
    *speed = handle_.curSpeeds;
    ulapi_mutex_give(handle_.handle);
    
    transformFromMount(handle_.curSpeeds, temp);
   
    speed->x = temp.x;
    speed->y = temp.y;
    speed->z = temp.z;
    speed->xrot = temp.xrot;
    speed->yrot = temp.yrot;
    speed->zrot = temp.zrot;

    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiUniversal::GetRobotSpeed (robotAxes *speed)
  {
    //! TODO
    return CANON_FAILURE;
  }


  LIBRARY_API CanonReturn CrpiUniversal::GetRobotTorques (robotAxes *torques)
  {
    ulapi_mutex_take(handle_.handle);
    *torques = handle_.curAxes;
    ulapi_mutex_give(handle_.handle);

    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiUniversal::MoveAttractor (robotPose &pose)
  {
    robotPose temp, temp2, temp3;
    robotPose curPose;
    vector<double> target;
#ifdef UNIVERSAL_NOISY
    cout << "target: " << endl;
    pose.print();
#endif

    GetRobotPose(&curPose);
#ifdef UNIVERSAL_NOISY
    cout << "current: " << endl;
    curPose.print();
#endif

    transformToMount(curPose, temp2);
#ifdef UNIVERSAL_NOISY
    cout << "current at base: " << endl;
    temp2.print();
#endif

    //! Get target pose relative to current pose, these become the X, Y and Z offsets used during force control
    transformToMount(pose, temp3);
#ifdef UNIVERSAL_NOISY
    cout << "target at base: " << endl;
    temp3.print();
#endif
    temp = temp2 - temp3;
#ifdef UNIVERSAL_NOIYS
    cout << "offset: " << endl;
    temp.print();
#endif

    target.push_back (temp.x);
    target.push_back (temp.y);
    target.push_back (temp.z);
    target.push_back (temp.xrot);
    target.push_back (temp.yrot);
    target.push_back (temp.zrot);

    if (generateParameter ('F', 'E', target))
    {
      if (send())
      {
        //! Okay
        
      }
      else
      {
        //! Oops
      }
    }
    else
    {
      cout << "Bad force command" << endl;
    }
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiUniversal::MoveToAxisTarget (robotAxes &axes)
  {
    robotAxes cur;
    double dist, dist2;
    int count;
    ulapi_real tim;

    //! Construct message
    vector<double> target;

    for (int i = 0; i < axes.axes; ++i)
    {
      //! Set axes to correct units
      if (angleUnits_ == DEGREE)
      {
        target.push_back (axes.axis.at(i) * (3.141592654f/180.0f));
      }
      else
      {
        target.push_back (axes.axis.at(i));
      }
    }

    //! PTP, Angular, Absolute
    if (generateMove ('P', 'A', 'A', target))
    {
      //! Send message to robot
      if (!send())
      {
        //! Error sending
        return CANON_FAILURE;
      }

#ifdef BLOCKING_MOTION     
      //! ROBOT DOES NOT BLOCK:  WAIT FOR RESPONSE
      dist2 = 1000.0;
      count = 0;
      tim = ulapi_time();
      while (true)
      {

        GetRobotAxes(&cur);
        dist = cur.distance(axes);
        //cout << dist << endl;

#ifdef VERIFY_MOVING
        if (dist >= dist2)
        {
          ++count;

          if (count >= 30)
          {
            //! Robot is not moving.  Retry.
            return CANON_FAILURE;
          }
        }
#endif

#ifdef USE_TIMEOUT
        if ((ulapi_time() - tim) > timethresh)
        {
          return CANON_FAILURE;
        }
#endif
        //printf("d %f\n", dist);
        if (dist <= distthresh)
        {
          break;
        }
        Sleep(100);
//        ulapi_wait(200000);
        dist2 = dist;
      }
#endif
    }
    else
    {
      //! Error generating motion message
      return CANON_FAILURE;
    }
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiUniversal::SetAbsoluteAcceleration (double acceleration)
  {
    if (acceleration > maxAccel_ || acceleration < 0.0f)
    {
      return CANON_REJECT;
    }

    acceleration_ = acceleration;
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiUniversal::SetAbsoluteSpeed (double speed)
  {
    if (speed > maxSpeed_ || speed < 0.0f)
    {
      return CANON_FAILURE;
    }

    speed_ = speed;
    return CANON_SUCCESS;
  }

  //  ("degree" or "radian")
  LIBRARY_API CanonReturn CrpiUniversal::SetAngleUnits (const char *unitName)
  {
    if (strcmp(unitName, "degree") == 0)
    {
      angleUnits_ = DEGREE;
    }
    else if (strcmp(unitName, "radian") == 0)
    {
      angleUnits_ = RADIAN;
    }
    else
    {
      CANON_FAILURE;
    }

    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiUniversal::SetAxialSpeeds (double *speeds)
  {
    //! TODO
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiUniversal::SetAxialUnits (const char **unitNames)
  {
    //! Not yet implemented
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiUniversal::SetEndPoseTolerance (robotPose &tolerances)
  {
    //! Not yet implemented
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiUniversal::SetIntermediatePoseTolerance (robotPose *tolerances)
  {
    //! Not yet implemented
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiUniversal::SetLengthUnits (const char *unitName)
  {
    if (strcmp(unitName, "meter") == 0)
    {
      lengthUnits_ = METER;
    }
    else if (strcmp(unitName, "mm") == 0)
    {
      lengthUnits_ = MM;
    }
    else if (strcmp(unitName, "inch") == 0)
    {
      lengthUnits_ = INCH;
    }
    else
    {
      CANON_FAILURE;
    }

    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiUniversal::SetParameter (const char *paramName, void *paramVal)
  {
    //! Not yet implemented
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiUniversal::SetRelativeAcceleration (double percent)
  {
    if (percent > 1.0f || percent < 0.0f)
    {
      return CANON_FAILURE;
    }

    acceleration_ = maxAccel_ * percent;
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiUniversal::SetRelativeSpeed (double percent)
  {
    if (percent > 1.0f || percent < 0.0f)
    {
      return CANON_FAILURE;
    }

    speed_ = maxSpeed_ * percent;
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiUniversal::SetRobotIO (robotIO &io)
  {
    bool changed = false;

    handle_.moveMe.str(string());
    handle_.moveMe << "def myProg():\n";
    for (int x = 0; x < io.ndio; ++x)
    {
      if (firstIO_ || (curIO_.dio[x] != io.dio[x]))
      {
        handle_.moveMe << "set_digital_out(" << x << ", ";
        handle_.moveMe << (io.dio[x] ? "True" : "False") << ")\n";
        changed = true;
      }
    }
    handle_.moveMe << "end\n";
    //! Send message to robot
    if (changed)
    {
      if (!send())
      {
        //! error sending
        return CANON_FAILURE;
      }
    }
    //timer_.waitUntil(10);

    curIO_ = io;
    firstIO_ = false;

    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiUniversal::SetRobotDO (int dig_out, bool val)
  {
    if (dig_out < 0 || dig_out >= curIO_.ndio)
    {
      return CANON_REJECT;
    }

    handle_.moveMe.str(string());
    handle_.moveMe << "def myProg():\n";
    handle_.moveMe << "set_digital_out(" << dig_out << ", ";
    handle_.moveMe << (val ? "True" : "False") << ")\n";
    handle_.moveMe << "end\n";
    //! Send message to robot
    if (!send())
    {
      //! error sending
      return CANON_FAILURE;
    }
    curIO_.dio[dig_out] = val;
    firstIO_ = false;

    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiUniversal::StopMotion (int condition)
  {
    ulapi_mutex_take(handle_.handle);
    handle_.moveMe.str(string());

    //! stopl(acceleration)
    handle_.moveMe << "def myProg():\n";
    handle_.moveMe << "stopl(3.0)\n";
    handle_.moveMe << "end\n";
    ulapi_mutex_give(handle_.handle);

    //! Send message to robot
    if (!send())
    {
      //! error sending
      return CANON_FAILURE;
    }
    return CANON_SUCCESS;
  }
  

  LIBRARY_API bool CrpiUniversal::generateMove (char moveType, char posType, char deltaType, vector<double> &input)
  {
    bool state = true;

    //! Check validity of inputs...
    //!   Check movement type
    state &= (moveType == 'P' || moveType == 'L' || moveType == 'F');
    //!   Check position type
    state &= (posType == 'C' || posType == 'A');
    //!   Check absolute or relative motion
    state &= (deltaType == 'A' || deltaType == 'R');
    //!   Check PTP-only angle movements
    state &= !(posType == 'A' && moveType == 'L');
    //!   Check for proper amount of input arguments
    //state &= (input.size() == 6);

    if (!state)
    {
      //! Invalid arguments generating move
      cout << "bad move" << endl;
      return false;
    }
    
    ulapi_mutex_take(handle_.handle);
    handle_.moveMe.str(string());
  
    handle_.moveMe << "def myProg():\n";
    handle_.moveMe << "move" << (moveType == 'P' ? "j(" : "l(");
    handle_.moveMe << (posType == 'C' ? "p[" : "[") << ((deltaType == 'A' ? 0.0f : curPose_[0]) + input.at(0)) << ", "
                   << ((deltaType == 'A' ? 0.0f : curPose_[1]) + input.at(1)) << ", " 
                   << ((deltaType == 'A' ? 0.0f : curPose_[2]) + input.at(2)) << ", "
                   << ((deltaType == 'A' ? 0.0f : curPose_[3]) + input.at(3)) << ", "
                   << ((deltaType == 'A' ? 0.0f : curPose_[4]) + input.at(4)) << ", "
                   << ((deltaType == 'A' ? 0.0f : curPose_[5]) + input.at(5))
                   << "]" << ", v=" << speed_ << ")\n";
    handle_.moveMe << "end\n";
    //cout << handle_.moveMe.str().c_str() << endl;
    /*
    handle_.moveMe << "while(True):\nforcem" << (moveType == 'P' ? "j(" : "l(");
    handle_.moveMe << (posType == 'C' ? "p[" : "[") << ((deltaType == 'A' ? 0.0f : curPose_[0]) + input.at(0)) << ", "
                   << ((deltaType == 'A' ? 0.0f : curPose_[1]) + input.at(1)) << ", " 
                   << ((deltaType == 'A' ? 0.0f : curPose_[2]) + input.at(2)) << ", "
                   << ((deltaType == 'A' ? 0.0f : curPose_[3]) + input.at(3)) << ", "
                   << ((deltaType == 'A' ? 0.0f : curPose_[4]) + input.at(4)) << ", "
                   << ((deltaType == 'A' ? 0.0f : curPose_[5]) + input.at(5))
                   << "], a=" << acceleration_ << ", v=" << speed_ << ")\nsync()\nend\n";
    */
    ulapi_mutex_give(handle_.handle);

    return true;
  }


  LIBRARY_API bool CrpiUniversal::generateTool (char mode, double value)
  {
    if (!(mode == 'B' || mode == 'A'  || mode == 'D'))
    {
      //! Value must be between 0 and 1
      return false;
    }



    //! TODO:  Populate variables 
    //moveMe_.str(string());
    //moveMe_ << "(0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000)";
    return true;
  }


  LIBRARY_API bool CrpiUniversal::generateParameter (char paramType, char subType, vector<double> &input)
  {
    bool state = true;

    //! Check validity of inputs...
    //!   Check movement type
    state &= (paramType == 'A' || paramType == 'S' || paramType == 'F' || paramType == 'T');
    //!   Check position type
    state &= (subType == 'A' || subType == 'R' || subType == 'E' || subType == 'D' || subType == 'C');

    if (paramType == 'F')
    {
      state &= (subType == 'E' || subType == 'D');
    }
    else if (paramType == 'T')
    {
      state &= (subType == 'D' || subType == 'C');
    }
    else
    {
      state &= (subType == 'A' || subType == 'R');
    }

    if (!state)
    {
      //! Invalid arguments generating move
      return false;
    }

    ulapi_mutex_take(handle_.handle);
    handle_.moveMe.str(string());
    handle_.moveMe << "def myProg():\n";
    switch (paramType)
    {
    case 'A':
      break;
    case 'S':
      break;
    case 'T':
      if (subType == 'D')
      {
        //! Define TCP
        handle_.moveMe << "set_tcp(p[" << input.at(0) << ", " << input.at(1) << ", " << input.at(2)
                       << ", " << input.at(3) << ", " << input.at(4) << ", " << input.at(5) << "])\n";
      }
      else
      {
        //! Center of mass
        handle_.moveMe << "set_payload(" << input.at(0) << ", (" << input.at(1) << ", " << input.at(2)
                       << ", " << input.at(3) << "))\n";
      }
      break;
    case 'F':
      if (handle_.curTool < 0)
      {
        cout << "bad tool" << endl;
        //! Cannot initiate force control without tool definition
        return false;
      }

      if (subType == 'E')
      {
        handle_.moveMe << "  set_payload(" << params_.tools.at(handle_.curTool).mass << ", ("
                       << (params_.tools.at(handle_.curTool).centerMass.x / 1000.0f) << ", "
                       << (params_.tools.at(handle_.curTool).centerMass.y / 1000.0f) << ", "
                       << (params_.tools.at(handle_.curTool).centerMass.z / 1000.0f) << "))\n";
        matrix rot(3, 3);
        vector<double> euler;
        euler.push_back(params_.mounting->xrot);
        euler.push_back(params_.mounting->yrot);
        euler.push_back(params_.mounting->zrot);
        rot.rotEulerMatrixConvert(euler);
        handle_.moveMe << "  set_gravity([" << (9.82f * rot.at(2, 0)) << ", " << (9.82f * rot.at(2, 1)) << ", "
                       << (9.82f * rot.at(2, 2)) << "])\n";
        //handle_.moveMe << "  set_gravity([0.0, -6.943788591251898, -6.943788591251896])\n";
        handle_.moveMe << "  def Walk():\n";
        handle_.moveMe << "    thread Force_properties_calculation_thread():\n";
        handle_.moveMe << "      while (True):\n";
        //handle_.moveMe << "        force_mode(tool_pose(), [1, 1, 1, 0, 0, 0], [0.0, 0.0, 5.0, 0.0, 0.0, 0.0], 2, [0.2, 0.2, 0.2, 0.17453292519943295, 0.17453292519943295, 0.17453292519943295])\n";
        handle_.moveMe << "        force_mode(p[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [1, 1, 1, 0, 0, 0], [20.0, 20.0, 20.0, 0.0, 0.0, 0.0], 2, [0.1, 0.1, 0.015, 0.17453292519943295, 0.17453292519943295, 0.17453292519943295])\n";
        handle_.moveMe << "        sync()\n";
        handle_.moveMe << "      end\n";
        handle_.moveMe << "    end\n";
        handle_.moveMe << "    global thread_handler = run Force_properties_calculation_thread()\n";

        //! Enable force mode
        if (input.size() == 6)
        {
          handle_.moveMe << "    global pt = pose_trans(cloud, p[X,Y,Z,0,0,0])\n";
          handle_.moveMe << "    movel(pose_trans(p[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], pt), a=0.1, v=0.02)\n";
        }
        handle_.moveMe << "    kill thread_handler\n";
        handle_.moveMe << "    end_force_mode()\n";
        handle_.moveMe << "  end\n"; //! "Walk()"
        //! Program begins here
        handle_.moveMe << "  global cloud = tool_pose()\n";// p[0.0000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000]\n";
        handle_.moveMe << "  global X = " << input.at(0) << "\n";
        handle_.moveMe << "  global Y = " << input.at(1) << "\n";
        handle_.moveMe << "  global Z = " << input.at(2) << "\n";
        handle_.moveMe << "  Walk()\n";
      }
      else
      {
        //! Disable force mode
        handle_.moveMe << "end_force_mode()\n";
      }

      break;
    default:
      //! Shouldn't get here.  We checked for this already.
      break;
    }
    handle_.moveMe << "end\n";

    ulapi_mutex_give(handle_.handle);
    
    return true;
  }


  LIBRARY_API bool CrpiUniversal::send ()
  {
#ifndef NEWTCPIP
    ulapi_mutex_take(handle_.handle);
    ulapi_integer client = ulapi_socket_get_client_id (handle_.params.tcp_ip_port, handle_.params.tcp_ip_addr);
    ulapi_socket_set_nonblocking(client);
    ulapi_mutex_give(handle_.handle);
#endif

    int sent;
#ifndef NEWTCPIP
    if (client > 0)
#else
    if (handle_.clientID > 0)
#endif
    {
      ulapi_mutex_take(handle_.handle);
#ifdef UNIVERSAL_NOISY
      cout << handle_.moveMe.str().c_str() << endl;
#endif


//      cout << handle_.moveMe.str().c_str() << endl;
#ifndef NEWTCPIP
      sent = ulapi_socket_write (client, handle_.moveMe.str().c_str(), strlen(handle_.moveMe.str().c_str()) + 1);
#else
      sent = ulapi_socket_write(handle_.clientID, handle_.moveMe.str().c_str(), strlen(handle_.moveMe.str().c_str()) + 1);
#endif

      ulapi_mutex_give(handle_.handle);
    }
    else
    {
      cout << endl << "cannot connect" << endl;
      return false;
    }
#ifndef NEWTCPIP
    ulapi_socket_close(client);
#endif

    return true;
  }


  LIBRARY_API bool CrpiUniversal::get ()
  {
    int x = 0;
    if (serialUsed_)
    {
//      printf ("getting feedback...\n");
      x = ulapi_serial_read(serialID_, mssgBuffer_, 8192);
//      printf ("%d read\n", x);
      return true;
    }
    else
    {
      //! TODO
      return false;
    }
  }

  LIBRARY_API bool CrpiUniversal::transformToMount(robotPose &in, robotPose &out, bool scale)
  {
    matrix pintemp(4,4), r(3,3), rtmp1(3,3);
    matrix pouttemp(4, 4);
    vector<double> vtemp;
    robotPose tmp = in;

    if (scale)
    {
      if (lengthUnits_ == MM)
      {
        //! Convert units from mm
        tmp.x /= 1000.0f;
        tmp.y /= 1000.0f;
        tmp.z /= 1000.0f;
      }
      else if (lengthUnits_ == INCH)
      {
        //! Convert units from inches
        tmp.x /= 39.3701f;
        tmp.y /= 39.3701f;
        tmp.z /= 39.3701f;
      }
    
      if (angleUnits_ = DEGREE)
      {
        tmp.xrot /= (180.0f / 3.141592654f);
        tmp.yrot /= (180.0f / 3.141592654f);
        tmp.zrot /= (180.0f / 3.141592654f);
      }
    }

    vtemp.push_back(tmp.xrot);
    vtemp.push_back(tmp.yrot);
    vtemp.push_back(tmp.zrot);
    r.rotEulerMatrixConvert(vtemp);
    //! Copy rotation matrix to homogeneous transformation matrix
    for (int x = 0; x < 3; ++x)
    {
      for (int y = 0; y < 3; ++y)
      {
        pintemp.at(x, y) = r.at(x, y);
      }
    }
    pintemp.at(0, 3) = tmp.x;
    pintemp.at(1, 3) = tmp.y;
    pintemp.at(2, 3) = tmp.z;
    pintemp.at(3, 3) = 1.0f;
    pouttemp = *backward_ * pintemp; //JAM pouttemp.matrixMult(*backward_, pintemp, pouttemp); // 

    out.x = pouttemp.at(0,3);
    out.y = pouttemp.at(1,3);
    out.z = pouttemp.at(2,3);
    for (int x = 0; x < 3; ++x)
    {
      for (int y = 0; y < 3; ++y)
      {
        r.at(x, y) = pouttemp.at(x, y);
      }
    }
    
    r.rotMatrixAxisAngleConvert(vtemp);
    out.xrot = vtemp.at(0);
    out.yrot = vtemp.at(1);
    out.zrot = vtemp.at(2);

    return true;
  }


  LIBRARY_API bool CrpiUniversal::transformFromMount(robotPose &in, robotPose &out, bool scale)
  {
    matrix pintemp(4,4), r(3,3), rtmp1(3,3);
    matrix pouttemp(4, 4);
    vector<double> vtemp;

    vtemp.push_back (in.xrot);
    vtemp.push_back (in.yrot);
    vtemp.push_back (in.zrot);
    rtmp1.rotAxisAngleMatrixConvert (vtemp);

    //! Copy rotation matrix to homogeneous transformation matrix
    for (int x = 0; x < 3; ++x)
    {
      for (int y = 0; y < 3; ++y)
      {
        pintemp.at(x, y) = rtmp1.at(x, y);
      }
    }

    pintemp.at(0, 3) = in.x;
    pintemp.at(1, 3) = in.y;
    pintemp.at(2, 3) = in.z;
    pintemp.at(3, 3) = 1.0f;
    pouttemp = (*forward_ * pintemp); //JAM pouttemp.matrixMult(*forward_, pintemp, pouttemp); // 
    out.x = pouttemp.at(0,3);
    out.y = pouttemp.at(1,3);
    out.z = pouttemp.at(2,3);
    for (int x = 0; x < 3; ++x)
    {
      for (int y = 0; y < 3; ++y)
      {
        r.at(x, y) = pouttemp.at(x, y);
      }
    }
    r.rotMatrixEulerConvert(vtemp);
    out.xrot = vtemp.at(0);
    out.yrot = vtemp.at(1);
    out.zrot = vtemp.at(2);

    if (scale)
    {
      if (lengthUnits_ == MM)
      {
        //! Convert units to mm
        out.x *= 1000.0f;
        out.y *= 1000.0f;
        out.z *= 1000.0f;
      }
      else if (lengthUnits_ == INCH)
      {
        //! Convert units to inches
        out.x *= 39.3701f;
        out.y *= 39.3701f;
        out.z *= 39.3701f;
      }
    
      if (angleUnits_ = DEGREE)
      {
        out.xrot *= (180.0f / 3.141592654f);
        out.yrot *= (180.0f / 3.141592654f);
        out.zrot *= (180.0f / 3.141592654f);
      }
    }

    return true;
  }

} // crpi_robot
