///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Robot Interface
//  Workfile:        crpi_kuka_lwr.cpp
//  Revision:        1.0 - 13 March, 2014
//                   1.1 - 16 June, 2014   Updated to use ulapi serial drivers
//  Author:          J. Marvel
//
//  Description
//  ===========
//  KUKA LWR 4+ interface definitions.
//
///////////////////////////////////////////////////////////////////////////////

#include "crpi_kuka_lwr.h"
#include <fstream>

using namespace std;

//#define STATIC_ //! Uncomment this line if you want to hard code the serial port (debugging purposes only)

//#define LWR_NOISY

namespace crpi_robot
{
  void livemanLWR (void *param)
  {
    keepalive *ka = (keepalive*)param;
    robotPose pose;
    crpi_timer timer;

    while (ka->runThread)
    {
      ((CrpiKukaLWR*)ka->rob)->GetRobotPose (&pose);
      //! Don't slam your processor!  You don't need to poll at full speed.
      timer.waitUntil(5000);
    }
    return;
  }

  void observerLWR(void *param)
  {
    /*
      Connect to observer 
      Create broadcast server
      Loop
        Read values from observer
        Broadcast values
    
    */


    /*
      //! Run as client
    //cout << "Connect to which address (xxx.xxx.xxx.xxx)? : ";
    //cin >> inbuffer;

//    server = ulapi_socket_get_client_id (6009, inbuffer);
    //server = ulapi_socket_get_client_id (1025, inbuffer);
    server = ulapi_socket_get_client_id(1025, "169.254.152.3");
    if (server < 0)
    {
      cout << "could not connect" << endl;
      return;
    }
    else
    {
      cout << "connected..." << endl;
      cout << ulapi_socket_set_blocking(server) << endl;
      cout << server << endl;
    }
    
    //! ABB Test
    timer timmytime;

    int counter = 0;
    char buffer[63];
    while (true)
    {
      int jj;
      //      ulapi_socket_write(server, "1", 1);
      for ( jj = 0; jj < 62; ++jj)
      {
      
        get = ulapi_socket_read(server, inbuffer, REQUEST_MSG_SIZE);
        buffer[jj] = inbuffer[0];
      }


      //inbuffer[get] = '\0';
      buffer[jj] = '\0';
      //if (get > 2)
      {
        cout << buffer << endl;
      }  
    
    */

    keepalive *ka = (keepalive*)param;
    robotPose pose;
    crpi_timer timer;

    while (ka->runThread)
    {
      ((CrpiKukaLWR*)ka->rob)->GetRobotPose(&pose);
      //! Don't slam your processor!  You don't need to poll at full speed.
      timer.waitUntil(5000);
    }
    return;
  }


  LIBRARY_API CrpiKukaLWR::CrpiKukaLWR (CrpiRobotParams &params)
  {
    mssgBuffer_ = new char[8192];

#ifndef OLDSERIAL
    if (ULAPI_OK != ulapi_init())
    {
#ifdef LWR_NOISY
      printf("ulapi_init error\n");
#endif
    }
#endif

#ifndef STATIC_

    params_ = params;
    ka_.handle = ulapi_mutex_new(99);

    if (params_.use_serial)
    {
#ifdef OLDSERIAL
      serial_ = new serial();
#else
      if (NULL == (serialID_ = ulapi_serial_new()))
      {
#ifdef LWR_NOISY
        printf ("\nCannot create serial object\n");
#endif
      }
#endif

#ifdef LWR_NOISY
        printf ("%s : ", COMChannel_);
#endif

#ifdef OLDSERIAL
      serialData_.setChannel (params_.serial_port);
#else
      if (ulapi_serial_open(COMChannel_, serialID_) == ULAPI_OK)
      {
#ifdef LWR_NOISY
        printf ("Opened COM channel\n");
#endif
      }
      else
      {
#ifdef LWR_NOISY
        printf ("Could not open COM channel\n");
#endif
      }
#endif

#ifdef OLDSERIAL
      serialData_.setBaud (params_.serial_rate);
#else
      if (ulapi_serial_baud(serialID_, val) == ULAPI_OK)
      {
#ifdef LWR_NOISY
        printf ("Set BAUD rate successful\n");
#endif
      }
      else
      {
#ifdef LWR_NOISY
        printf ("Could not set BAUD rate\n");
#endif
      }
#endif

#ifdef OLDSERIAL
      serialData_.setParity(params_.serial_parity_even);
      serialData_.setStopBits(params_.serial_sbits);
#endif

#ifdef OLDSERIAL
      if (serial_->attach(serialData_))
      {
#ifdef LWR_NOISY
        printf ("serial connection to arm successful\n");
#endif
      }
      else
      {
#ifdef LWR_NOISY
        printf ("serial connection to arm failed\n");
#endif
      }
#else
      if (ulapi_serial_set_nonblocking(serialID_) == ULAPI_OK) //set_nonblocking
      {
#ifdef LWR_NOISY
        printf ("Set serial port blocking okay\n");
#endif
      }
      else
      {
#ifdef LWR_NOISY
        printf ("Could not set serial port blocking\n");
#endif
      }
      val = ulapi_serial_write(serialID_,
                                "TBx 1.00000000 0.00000000 0.00000000 0.00000000 0.00000000",
                                strlen("TBx 1.00000000 0.00000000 0.00000000 0.00000000 0.00000000"));
#endif        
    } // if (params_.use_serial)
    else
    {
      //! Use TCP

      //! Wait for connection from LWR
      printf ("Waiting for connection...\n");
      server_ = ulapi_socket_get_server_id(params_.tcp_ip_port);
      client_ = ulapi_socket_get_connection_id(server_);
      ulapi_socket_set_blocking(client_);

      task = ulapi_task_new();
      ka_.rob = this;
      ka_.runThread = true;

      ulapi_task_start((ulapi_task_struct*)task, livemanLWR, &ka_, ulapi_prio_lowest(), 0);
    }
#else
    bool test;
    test = serialData_.setBaud (57600);
    test &= serialData_.setChannel (1);
    serialUsed_ = true;
#endif

    feedback_ = new double[10];
    tempData_ = new vector<string>(10, " ");

    angleUnits_ = DEGREE;
    lengthUnits_ = MM;
  }


  LIBRARY_API CrpiKukaLWR::~CrpiKukaLWR ()
  {
    delete [] mssgBuffer_;
    delete [] feedback_;
    if (params_.use_serial)
    {
#ifdef OLDSERIAL
      serial_->closeConnection(serialData_);
#else
      ulapi_serial_close(serialID_);
      ulapi_serial_delete(serialID_);
#endif
    }
    else
    {
      ulapi_socket_close(server_);
    }
  }


  LIBRARY_API CanonReturn CrpiKukaLWR::ApplyCartesianForceTorque (robotPose &robotForceTorque, vector<bool> activeAxes, vector<bool> manipulator)
  {
    //! TODO
    return CANON_FAILURE;
  }


  LIBRARY_API CanonReturn CrpiKukaLWR::ApplyJointTorque (robotAxes &robotJointTorque)
  {
    //! TODO
    return CANON_FAILURE;
  }


  LIBRARY_API CanonReturn CrpiKukaLWR::SetTool (double percent)
  {
    ulapi_mutex_take(ka_.handle);
    if (generateTool ('B', percent))
    {
      //! Send message to robot
      if (!send ())
      {
        //! error sending
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      //! Wait for response from robot
      if (!get ())
      {
        //! error getting
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      ulapi_mutex_give(ka_.handle);
      if (mssgBuffer_[0] == '1')
      {
        return CANON_SUCCESS;
      }
      else
      {
        return CANON_FAILURE;
      }
    }
    else
    {
      //! Error generating tool actuation message
      ulapi_mutex_give(ka_.handle);
      return CANON_FAILURE;
    }
  }


  LIBRARY_API CanonReturn CrpiKukaLWR::Couple (const char *targetID)
  {
    std::vector<CrpiToolDef>::const_iterator itr;
    for (itr = params_.tools.begin(); itr != params_.tools.end(); ++itr)
    {
      if (strcmp(targetID, itr->toolName.c_str()) == 0)
      {
        break;
      }
    }
    if (itr == params_.tools.end())
    {
      return CANON_FAILURE;
    }

    ulapi_mutex_take(ka_.handle);
    if (generateTool('D', itr->toolID))
    {
      //! Send message to robot
      if (!send ())
      {
        //! error sending
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      //! Wait for response from robot
      if (!get ())
      {
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      ulapi_mutex_give(ka_.handle);
    }
    else
    {
      ulapi_mutex_give(ka_.handle);
      return CANON_FAILURE;
    }

    if (mssgBuffer_[0] == '1')
    {
      return CANON_SUCCESS;
    }
    else
    {
      return CANON_FAILURE;
    }
  }


  LIBRARY_API CanonReturn CrpiKukaLWR::Message (const char *message)
  {
    //! The KR C2 controller cannot display a message on the teach pendant, it seems
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiKukaLWR::MoveStraightTo (robotPose &pose)
  {
    //! Construct message
    vector<double> target;
    target.push_back (pose.x);
    target.push_back (pose.y);
    target.push_back (pose.z);
    target.push_back (pose.zrot);
    target.push_back (pose.yrot);
    target.push_back (pose.xrot);
    target.push_back (pose.status);
    target.push_back (pose.turns);
    target.push_back (0.0);
    target.push_back (0.0);

    ulapi_mutex_take(ka_.handle);
    //! LIN, Cartesian, Absolute
    if (generateMove ('L', 'C', 'A', target))
    {
      //! Send message to robot
      if (!send ())
      {
        //! error sending
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      //! Wait for response from robot
      if (!get ())
      {
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      ulapi_mutex_give(ka_.handle);

      if (mssgBuffer_[0] == '1')
      {
        return CANON_SUCCESS;
      }
      else
      {
        return CANON_FAILURE;
      }
    }
    else
    {
      //! Error generating motion message
      ulapi_mutex_give(ka_.handle);
      return CANON_FAILURE;
    }
  }


  LIBRARY_API CanonReturn CrpiKukaLWR::MoveThroughTo (robotPose *poses,
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


  LIBRARY_API CanonReturn CrpiKukaLWR::MoveTo (robotPose &pose)
  {
    //! Construct message
    vector<double> target;
    target.push_back (pose.x);
    target.push_back (pose.y);
    target.push_back (pose.z);
    target.push_back (pose.zrot);
    target.push_back (pose.yrot);
    target.push_back (pose.xrot);
    target.push_back (pose.status);
    target.push_back (pose.turns);
    target.push_back (0.0);
    target.push_back (0.0);

    //! PTP, Cartesian, Absolute
    //if (generateMove ('P', 'C', 'R', target)) //! JAM:  for initial testing purposes only
    ulapi_mutex_take(ka_.handle);
    if (generateMove ('P', 'C', 'A', target))
    {
      //! Send message to robot
      if (!send())
      {
        //! error sending
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      //! Wait for response from robot
      if (!get ())
      {
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      ulapi_mutex_give(ka_.handle);
      if (mssgBuffer_[0] == '1')
      {
        return CANON_SUCCESS;
      }
      else
      {
        return CANON_FAILURE;
      }
    }
    else
    {
      //! Error generating motion message
      ulapi_mutex_give(ka_.handle);
      return CANON_FAILURE;
    }
  }


  LIBRARY_API CanonReturn CrpiKukaLWR::GetRobotAxes (robotAxes *axes)
  {
    //! Construct request for axis information

    ulapi_mutex_take(ka_.handle);
    if (generateFeedback ('A'))
    {
      if (!send ())
      {
        //! error sending
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      //! Wait for response from robot
      if (!get ())
      {
        //! error receiving
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      
      //! Parse data
      if (!parseFeedback (7))
      {
        //! Error parsing data
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }

      ulapi_mutex_give(ka_.handle);
      try
      {
        for (int i = 0; i < axes->axes; ++i)
        {
          axes->axis.at(i) = feedback_[i];
        }
      }
      catch (...)
      {
        //! probably an axis violation due to vector missmatch
        return CANON_FAILURE;
      }
    } // if generateFeedback ('A')
    else
    {
      //! Error generating feedback request message
      ulapi_mutex_give(ka_.handle);
      return CANON_FAILURE;
    }
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiKukaLWR::GetRobotForces (robotPose *forces)
  {
    //! Construct request for axis information
    ulapi_mutex_take(ka_.handle);
    if (generateFeedback('F'))
    {
      if (!send())
      {
        //! error sending
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      //! Wait for response from robot
      if (!get())
      {
        //! error receiving
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      //! Parse data
      if (!parseFeedback(6))
      {
        //! Error parsing data
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      ulapi_mutex_give(ka_.handle);

      try
      {
        forces->x = feedback_[0];
        forces->y = feedback_[1];
        forces->z = feedback_[2];
        forces->zrot = feedback_[3];
        forces->yrot = feedback_[4];
        forces->xrot = feedback_[5];
      }
      catch (...)
      {
        //! probably an axis violation due to vector missmatch
        return CANON_FAILURE;
      }
    }
    else
    {
      //! Error generating feedback request message
      ulapi_mutex_give(ka_.handle);
      return CANON_FAILURE;
    }

    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiKukaLWR::GetRobotIO (robotIO *io)
  {
    //! Construct request for axis information
    ulapi_mutex_take(ka_.handle);
    if (generateFeedback('S'))
    {
      if (!send())
      {
        //! error sending
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      //! Wait for response from robot
      if (!get())
      {
        //! error receiving
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      //! Parse data
      if (!parseFeedback(8))
      {
        //! Error parsing data
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      ulapi_mutex_give(ka_.handle);

      try
      {
        io->dio[0] = (feedback_[1] > 0.5f);
        io->dio[1] = (feedback_[2] > 0.5f);
        io->dio[2] = (feedback_[3] > 0.5f);
        io->dio[3] = (feedback_[4] > 0.5f);
        io->dio[4] = (feedback_[5] > 0.5f);
        io->dio[5] = (feedback_[6] > 0.5f);
        io->dio[6] = (feedback_[7] > 0.5f);
      }
      catch (...)
      {
        //! probably an axis violation due to vector missmatch
        return CANON_FAILURE;
      }
    }
    else
    {
      //! Error generating feedback request message
      ulapi_mutex_give(ka_.handle);
      return CANON_FAILURE;
    }

    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiKukaLWR::GetRobotPose (robotPose *pose)
  {
    //! Construct request for axis information
    ulapi_mutex_take(ka_.handle);
    if (generateFeedback ('C'))
    {
      if (!send ())
      {
        //! error sending
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      //! Wait for response from robot
      if (!get ())
      {
        //! error receiving
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      //! Parse data
      if (!parseFeedback (8))
      {
        //! Error parsing data
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      ulapi_mutex_give(ka_.handle);

      try
      {
        pose->x = feedback_[0];
        pose->y = feedback_[1];
        pose->z = feedback_[2];
        pose->zrot = feedback_[3];
        pose->yrot = feedback_[4];
        pose->xrot = feedback_[5];
        pose->status = (int)feedback_[6];
        pose->turns = (int)feedback_[7];

        if (lengthUnits_ == METER)
        {
          pose->x /= 1000.0f;
          pose->y /= 1000.0f;
          pose->z /= 1000.0f;
        }
        else if (lengthUnits_ == INCH)
        {
          pose->x /= 25.4f;
          pose->y /= 25.4f;
          pose->z /= 25.4f;
        }

        if (angleUnits_ == RADIAN)
        {
          pose->zrot *= (3.141592654f / 180.0f);
          pose->yrot *= (3.141592654f / 180.0f);
          pose->xrot *= (3.141592654f / 180.0f);
        }
      }
      catch (...)
      {
        //! probably an axis violation due to vector missmatch
        return CANON_FAILURE;
      }
    }
    else
    {
      //! Error generating feedback request message
      ulapi_mutex_give(ka_.handle);
      return CANON_FAILURE;
    }

    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiKukaLWR::GetRobotSpeed (robotPose *speed)
  {
    //! TODO
    return CANON_FAILURE;
  }


  LIBRARY_API CanonReturn CrpiKukaLWR::GetRobotSpeed (robotAxes *speed)
  {
    //! TODO
    return CANON_FAILURE;
  }


  LIBRARY_API CanonReturn CrpiKukaLWR::GetRobotTorques (robotAxes *torques)
  {
    //! Construct request for axis information
    ulapi_mutex_take(ka_.handle);
    if (generateFeedback('T'))
    {
      if (!send())
      {
        //! error sending
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      //! Wait for response from robot
      if (!get())
      {
        //! error receiving
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      //! Parse data
      if (!parseFeedback(6))
      {
        //! Error parsing data
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      ulapi_mutex_give(ka_.handle);

      try
      {
        torques->axis.at(0) = feedback_[0];
        torques->axis.at(1) = feedback_[1];
        torques->axis.at(2) = 0.0f;
        torques->axis.at(3) = feedback_[2];
        torques->axis.at(4) = feedback_[3];
        torques->axis.at(5) = feedback_[4];
        torques->axis.at(6) = feedback_[5];

        if (angleUnits_ == RADIAN)
        {
          for (int i = 0; i < 7; ++i)
          {
            torques->axis.at(i) *= (3.141592654f / 180.0f);
          }
        }
      }
      catch (...)
      {
        //! probably an axis violation due to vector missmatch
        return CANON_FAILURE;
      }
    }
    else
    {
      //! Error generating feedback request message
      ulapi_mutex_give(ka_.handle);
      return CANON_FAILURE;
    }

    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiKukaLWR::MoveAttractor (robotPose &pose)
  {
    //! Construct message
    vector<double> target;
    target.push_back (pose.x);
    target.push_back (pose.y);
    target.push_back (pose.z);
    target.push_back (pose.zrot);
    target.push_back (pose.yrot);
    target.push_back (pose.xrot);
    target.push_back (pose.status);
    target.push_back (pose.turns);
    target.push_back (0.0);
    target.push_back (0.0);

    ulapi_mutex_take(ka_.handle);
    if (generateMove ('L', 'F', 'A', target))
    {
      //! Send message to robot
      if (!send ())
      {
        //! error sending
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      //! Wait for response from robot
      if (!get ())
      {
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      ulapi_mutex_give(ka_.handle);

      if (mssgBuffer_[0] == '1')
      {
        return CANON_SUCCESS;
      }
      else
      {
        return CANON_FAILURE;
      }
    }
    else
    {
      //! Error generating motion message
      ulapi_mutex_give(ka_.handle);
      return CANON_FAILURE;
    }

    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiKukaLWR::MoveToAxisTarget (robotAxes &axes)
  {
    //! Construct message
    vector<double> target;
    for (int i = 0; i < axes.axes; ++i)
    {
      target.push_back (axes.axis.at(i));
    }

    //! PTP, Angular, Absolute
    ulapi_mutex_take(ka_.handle);
    if (generateMove ('P', 'A', 'A', target))
    {
      //! Send message to robot
      if (!send())
      {
        //! error sending
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      //! Wait for response from robot
      if (!get ())
      {
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      ulapi_mutex_give(ka_.handle);
    }
    else
    {
      //! Error generating motion message
      ulapi_mutex_give(ka_.handle);
      return CANON_FAILURE;
    }

    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiKukaLWR::SetAbsoluteAcceleration (double tolerance)
  {
    //! Not yet implemented
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiKukaLWR::SetAbsoluteSpeed (double speed)
  {
    //! Not yet implemented
    return CANON_REJECT;
  }

//  ("degree" or "radian")
  LIBRARY_API CanonReturn CrpiKukaLWR::SetAngleUnits (const char *unitName)
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

    //! Not yet implemented
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiKukaLWR::SetAxialSpeeds (double *speeds)
  {
    //! Not yet implemented
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiKukaLWR::SetAxialUnits (const char **unitNames)
  {
    //! Not yet implemented
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiKukaLWR::SetEndPoseTolerance (robotPose &tolerances)
  {
    //! Not yet implemented
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiKukaLWR::SetIntermediatePoseTolerance (robotPose *tolerances)
  {
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiKukaLWR::SetLengthUnits (const char *unitName)
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
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiKukaLWR::SetParameter (const char *paramName, void *paramVal)
  {
    //! Not yet implemented
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiKukaLWR::SetRelativeAcceleration (double percent)
  {
    //! Not yet implemented
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiKukaLWR::SetRelativeSpeed (double percent)
  {
    //! Not yet implemented
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiKukaLWR::SetRobotIO (robotIO &io)
  {
    bool state = true;
    for (int x = 0; x < 7; ++x)
    {
      state &= (SetRobotDO(x, io.dio[x]) == CANON_SUCCESS);
    }

    return (state ? CANON_SUCCESS : CANON_FAILURE);
  }


  LIBRARY_API CanonReturn CrpiKukaLWR::SetRobotDO (int dig_out, bool val)
  {
    //! Construct digital signal output command
    ulapi_mutex_take(ka_.handle);
    if (generateIO('D', dig_out, val))
    {
      //! Send message to robot
      if (!send())
      {
        //! error sending
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      //! Wait for response from robot
      if (!get())
      {
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      ulapi_mutex_give(ka_.handle);
    }
    else
    {
      //! Error generating feedback request message
      ulapi_mutex_give(ka_.handle);
      return CANON_FAILURE;
    }
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiKukaLWR::StopMotion (int condition)
  {
    //! Not yet implemented
    return CANON_REJECT;
  }
  

  LIBRARY_API bool CrpiKukaLWR::generateMove (char moveType, char posType, char deltaType, vector<double> &input)
  {
    bool state = true;
    size_t found;
    int extension = 0,
        currLength = 0,
        i = 0,
        j = 0;

    //! Check validity of inputs...
    //!   Check movement type
    state &= (moveType == 'P' || moveType == 'L');
    //!   Check position type
    state &= (posType == 'C' || posType == 'A' || posType == 'F');
    //!   Check absolute or relative motion
    state &= (deltaType == 'A' || deltaType == 'R');
    //!   Check PTP-only angle movements
    //state &= !(posType == 'A' && moveType == 'L');
    //!   Check for proper amount of input arguments
    //state &= (input.size() == 6);

    if (!state)
    {
      //! Invalid arguments generating move
      return false;
    }

    //! Clear variables
    moveMe_.str(string());
    tempString_.str(string());

    if (moveType == 'F')
    {
      //! Overwrite any attempt to do joint angle force motion
      posType = 'C';
    }

    if (params_.use_serial)
    {
      moveMe_ << moveType << posType << deltaType;
    }
    else
    {
      moveMe_ << "<?xml version=\"1.0\"?><CRPIData><CMD>" << moveType << posType << deltaType << "</CMD><Values>";
    }

    //! Loop through paramaters, completes 10 char string, and adds to command string
    for (i = 0; i < 10 ; ++i)
    {
      if (params_.use_serial)
      {
        //! Adds space delimiter and number to string
        moveMe_ << " ";
      }
      else
      {
        moveMe_ << "<V" << (i+1) << ">";
      }
      
      tempString_.str(string());
      tempString_ << input[i];

      //! If number is an integer, add a decimal point
      found = tempString_.str().find('.');
      if (found == string::npos)
      {
        tempString_ << ".";
      }

      //! Add trailing 0s to decimal to create 10-char number string
      tempString_.seekg (0, ios::end);
      currLength = (int) tempString_.tellg ();

      for (j = currLength; j < 6; ++j)
      {
        tempString_ << "0";
      }

      moveMe_ << tempString_.str();
      if (params_.use_serial)
      {
        moveMe_ << "\0";
      }
      else
      {
        moveMe_ << "</V" << (i+1) << ">";
      }
    } // for (i = 0; i < 10; ++i)

    if (!params_.use_serial)
    {
      moveMe_ << "</Values></CRPIData>";
    }

    return true;
  }


  LIBRARY_API bool CrpiKukaLWR::generateTool (char mode, double value)
  {
    if (!(mode == 'B' || mode == 'A'  || mode == 'D'))
    {
      return false;
    }
    size_t found;
    int currLength, j;

    moveMe_.str(string());

    if (!params_.use_serial)
    {
      moveMe_ << "<?xml version=\"1.0\"?><CRPIData><CMD>";
    }
    moveMe_ << 'T' << mode << "x ";
    if (params_.use_serial)
    {
      moveMe_ << " ";
    }
    else
    {
      moveMe_ << "</CMD><Values><V1>";
    }

    //! Convert the input value to a character string of length 10
    tempString_.str(string());
    tempString_ << value;

    //! If number is an integer, add a decimal point
    found = tempString_.str().find('.');
    if (found == string::npos)
    {
      tempString_ << ".";
    }

    //! Add trailing 0s to decimal to create 10-char number string
    tempString_.seekg (0, ios::end);
    currLength = (int) tempString_.tellg ();

    for (j = currLength; j < 6; ++j)
    {
      tempString_ << "0";
    }

    moveMe_ << tempString_.str();
    if (params_.use_serial)
    {
      moveMe_ << " 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000\0";
    }
    else
    {
      moveMe_ << "</V1><V2>0.00000000</V2><V3>0.00000000</V3><V4>0.00000000</V4><V5>0.00000000</V5><V6>0.00000000</V6><V7>0.00000000</V7><V8>0.00000000</V8><V9>0.00000000</V9><V10>0.00000000</V10></Values></CRPIData>";
    }
#ifdef LWR_NOISY
    printf(moveMe_.str().c_str());
#endif
    return true;
  }


  LIBRARY_API bool CrpiKukaLWR::generateFeedback (char retType)
  {
    if (retType != 'C' && retType != 'A' && retType != 'F' && retType != 'T' && retType != 'S')
    {
      //! Unsupported variable
      return false;
    }

    moveMe_.str(string());

    if (!params_.use_serial)
    {
      moveMe_ << "<?xml version=\"1.0\"?><CRPIData><CMD>";
    }

    moveMe_ << retType;

    if (params_.use_serial)
    {
      moveMe_ << "xx 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000\0";
    }
    else
    {
      moveMe_ << "xx</CMD><Values><V1>0.00000000</V1><V2>0.00000000</V2><V3>0.00000000</V3><V4>0.00000000</V4><V5>0.00000000</V5><V6>0.00000000</V6><V7>0.00000000</V7><V8>0.00000000</V8><V9>0.00000000</V9><V10>0.00000000</V10></Values></CRPIData>";
    }
    return true;
  }


  LIBRARY_API bool CrpiKukaLWR::generateIO(char sigtype, int signum, double val)
  {
    if (sigtype != 'D' && sigtype != 'A')
    {
      //! Unsupported variable
      return false;
    }

    moveMe_.str(string());

    if (!params_.use_serial)
    {
      moveMe_ << "<?xml version=\"1.0\"?><CRPIData><CMD>";
    }

    moveMe_ << "I" << sigtype;

    if (params_.use_serial)
    {
      moveMe_ << "x " << signum << " " << val << " 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000\0";
    }
    else
    {
      moveMe_ << "x</CMD><Values><V1>" << signum << "</V1><V2>" << val << "</V2><V3>0.00000000</V3><V4>0.00000000</V4><V5>0.00000000</V5><V6>0.00000000</V6><V7>0.00000000</V7><V8>0.00000000</V8><V9>0.00000000</V9><V10>0.00000000</V10></Values></CRPIData>";
    }
    return true;
  }


  LIBRARY_API bool CrpiKukaLWR::generateParameter (char paramType, char subType, vector<double> &input)
  {
    size_t found;
    int j = 0, currLength = 0;
    
    switch (paramType)
    {
    case 'A':
      //! Set acceleration
      //! JAM:  TODO
      if (subType == 'A')
      {
        //! Absolute
      }
      else if (subType == 'R')
      {
        //! Relative
      }
      else
      {
        return false;
      }
      break;
    case 'S':
      //! Set speed
      if (subType == 'A')
      {
        //! Absolute
        if (input[0] > 2.0 || input[0] < 0.0)
        {
          return false;
        }
      }
      else if (subType == 'R')
      {
        //! Relative
        if (input[0] > 100 || input[0] < 0)
        {
          return false;
        }
      }
      else
      {
        return false;
      }

      moveMe_.str (string());
      if (!params_.use_serial)
      {
        moveMe_ << "<?xml version=\"1.0\"?><CRPIData><CMD>";
      }
      moveMe_ << 'V' << paramType << subType;
      if (params_.use_serial)
      {
        moveMe_ << " ";
      }
      else
      {
        moveMe_ << "</CMD><Values><V1>";
      }

      tempString_.str(string());
      tempString_ << input[0];

      //! If number is an integer, add a decimal point
      found = tempString_.str().find('.');
      if (found == string::npos)
      {
        tempString_ << ".";
      }

      //! Add trailing 0s to decimal to create 10-char number string
      tempString_.seekg (0, ios::end);
      currLength = (int) tempString_.tellg ();

      for (j = currLength; j < 6; ++j)
      {
        tempString_ << "0";
      }

      if (params_.use_serial)
      {
        moveMe_ << tempString_.str() << " 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000\0";
      }
      else
      {
        moveMe_ << tempString_.str() << "</V1><V2>0.00000000</V2><V3>0.00000000</V3><V4>0.00000000</V4><V5>0.00000000</V5><V6>0.00000000</V6><V7>0.00000000</V7><V8>0.00000000</V8><V9>0.00000000</V9><V10>0.00000000</V10></Values></CRPIData>";
      }

      break;
    default:
      return false;
    }
    
    return true;
  }

  
  LIBRARY_API bool CrpiKukaLWR::send ()
  {
    int x;
    
#ifdef LWR_NOISY
    printf ("Sending message %s\n", moveMe_.str().c_str());
    printf ("fflushhhhhh...\n");
#endif
    //fflush(NULL);
#ifdef LWR_NOISY
    printf ("flushed\n");
#endif
    if (params_.use_serial)
    {
      //! Use Serial
#ifdef OLDSERIAL
#ifdef LWR_NOISY
    printf ("Sending...\n");
#endif
      serial_->sendData(moveMe_.str().c_str(), serialData_);
#ifdef LWR_NOISY
    printf ("Sent!\n");
#endif
#else
      x = ulapi_serial_write(serialID_, moveMe_.str().c_str(), strlen(moveMe_.str().c_str())+1);
#ifdef LWR_NOISY
      printf ("%i\n", x);
#endif
#endif
      return true;
    }
    else
    {
      //! Use TCP/IP
      x = ulapi_socket_write(client_, moveMe_.str().c_str(), strlen(moveMe_.str().c_str())+1);
#ifdef LWR_NOISY
      printf ("%i\n", x);
#endif
      return true;
    }
  }


  LIBRARY_API bool CrpiKukaLWR::get ()
  {
    int x = 0;

#ifdef LWR_NOISY
      printf ("getting feedback...\n");
#endif

    if (params_.use_serial)
    {
      //! Use serial
#ifdef OLDSERIAL
      serial_->getData (mssgBuffer_, serialData_, 285);
#else
      x = ulapi_serial_read(serialID_, mssgBuffer_, 8192);
#endif
#ifdef LWR_NOISY
      printf ("%d %s read\n", x, mssgBuffer_);
#endif
      return true;
    }
    else
    {
      //! Use TCP/IP
      x = ulapi_socket_read(client_, mssgBuffer_, 8192);
#ifdef LWR_NOISY
      printf ("%d %s read\n", x, mssgBuffer_);
#endif
      return true;
    }
  }


  LIBRARY_API bool CrpiKukaLWR::parseFeedback (int num)
  {
    bool newItem = false;
    int index = -1, i;

    for (i = 0; i < num; ++i)
    {
      tempData_->at(i) = "";
    }

    //! REQUEST_MSG_SIZE is very large, feedback from robot is limited to 80 characters
    for (i = 0; i < 80; ++i)
    {
      if (mssgBuffer_[i] == ' ')
      {
        newItem = false;
      }
      else
      {
        if (!newItem)
        {
          newItem = true;
          index++;
          if( index >= num )
          {
            break;
//            return false;
          }
        }
        tempData_->at(index).push_back (mssgBuffer_[i]);
      } // if (mssgBuffer_[i] == ' ') ... else
    } //for (int i = 0; i < 65; ++i)
  
    //! Convert strings to floats
    for (i = 0; i < num; ++i)
    {
      feedback_[i] = stod (tempData_->at(i));
    }
    return true;
  }

} // crpi_robot
