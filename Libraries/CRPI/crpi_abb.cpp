///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Robot Interface
//  Workfile:        crpi_abb.cpp
//  Revision:        1.0 - 10 February, 2016
//  Author:          J. Marvel
//
//  Description
//  ===========
//  ABB IRB 14000 interface definitions.
//
///////////////////////////////////////////////////////////////////////////////

#include "crpi_abb.h"
#include "..\Math\MatrixMath.h"
#include <fstream>

using namespace std;

//#define ABB_NOISY

namespace crpi_robot
{
  void livemanABB (void *param)
  {
    keepalive *ka = (keepalive*)param;
    robotPose pose;
    crpi_timer timer;

    while (ka->runThread)
    {
      ((CrpiAbb*)ka->rob)->GetRobotPose (&pose);

      //! Don't slam your processor!  You don't need to poll at full speed.
      timer.waitUntil(5000);
    }
    return;
  }


  LIBRARY_API CrpiAbb::CrpiAbb (CrpiRobotParams &params)
  {
    mssgBuffer_ = new char[8192];

    params_ = params;
    ka_.handle = ulapi_mutex_new(99);

    //! Connect to ABB IRB 14000 server
    server_ = ulapi_socket_get_client_id (params_.tcp_ip_port, params_.tcp_ip_addr);
    ulapi_socket_set_blocking(server_);

    task = ulapi_task_new();
    ka_.rob = this;
    ka_.runThread = true;

    ulapi_task_start((ulapi_task_struct*)task, livemanABB, &ka_, ulapi_prio_lowest(), 0);

    feedback_ = new double[10];
    tempData_ = new vector<string>(10, " ");

    angleUnits_ = DEGREE;
    lengthUnits_ = MM;
    curTool_ = 1; //! Set default to parallel gripper
  }


  LIBRARY_API CrpiAbb::~CrpiAbb ()
  {
    delete [] mssgBuffer_;
    delete [] feedback_;
    ulapi_socket_close(server_);
  }


  LIBRARY_API CanonReturn CrpiAbb::SetTool (double percent)
  {
    bool status = false;
    ulapi_mutex_take(ka_.handle);
    if (curTool_ == 1)
    {
      status = generateTool ('A', percent);
    }
    else if (curTool_ == 2)
    {
      status = generateTool ('B', percent);
    }
    else
    {
      return CANON_FAILURE;
    }

    if (status)
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

  LIBRARY_API CanonReturn CrpiAbb::ApplyCartesianForceTorque (robotPose &robotForceTorque, vector<bool> activeAxes, vector<bool> manipulator)
  {
    return CANON_SUCCESS;
  }

  LIBRARY_API CanonReturn CrpiAbb::ApplyJointTorque (robotAxes &robotJointTorque)
  {
    return CANON_SUCCESS;
  }

  LIBRARY_API CanonReturn CrpiAbb::Couple (const char *targetID)
  {
    std::vector<CrpiToolDef>::const_iterator itr;
    for (itr = params_.tools.begin(); itr != params_.tools.end(); ++itr)
    {
      if (strcmp(targetID, itr->toolName.c_str()) == 0)
      {
        curTool_ = itr->toolID;
        break;
      }
    }
    if (itr == params_.tools.end())
    {
      return CANON_FAILURE;
    }
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiAbb::Message (const char *message)
  {
    //! The KR C2 controller cannot display a message on the teach pendant, it seems
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiAbb::MoveStraightTo (robotPose &pose)
  {
    //! Construct message
    vector<double> target;
    target.push_back (pose.x);
    target.push_back (pose.y);
    target.push_back (pose.z);

    Math::matrix m1(3,3);
    vector<double> q, e;
    e.push_back(pose.xrot);
    e.push_back(pose.yrot);
    e.push_back(pose.zrot);

    if (angleUnits_ == DEGREE)
    {
      e.at(0) *= (3.141592654f / 180.0f);
      e.at(1) *= (3.141592654f / 180.0f);
      e.at(2) *= (3.141592654f / 180.0f);
    }

    m1.rotEulerMatrixConvert(e);
    m1.rotMatrixQuaternionConvert(q);

    target.push_back (q.at(0));
    target.push_back (q.at(1));
    target.push_back (q.at(2));
    target.push_back (q.at(3));

    ulapi_mutex_take(ka_.handle);
    //! LIN, Cartesian, Absolute
    if (generateMove ('L', 'C', 'A', target))
    {
      //! Send message to robot
      if (!send ())
      {
        //! error sending
        printf("failed send\n");
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      //! Wait for response from robot
      if (!get ())
      {
        printf("failed get\n");
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      ulapi_mutex_give(ka_.handle);
//      printf("%s\n", mssgBuffer_);
      if (mssgBuffer_[1] == '1')
      {
        return CANON_SUCCESS;
      }
      else
      {
        printf("bad motion?\n");
        return CANON_FAILURE;
      }
    }
    else
    {
      //! Error generating motion message
      printf("bad message\n");
      ulapi_mutex_give(ka_.handle);
      return CANON_FAILURE;
    }
  }


  LIBRARY_API CanonReturn CrpiAbb::MoveThroughTo (robotPose *poses,
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


  LIBRARY_API CanonReturn CrpiAbb::MoveTo (robotPose &pose)
  {
    //! Construct message
    vector<double> target;
    target.push_back (pose.x);
    target.push_back (pose.y);
    target.push_back (pose.z);

    Math::matrix m1(3,3);
    vector<double> q, e;
    e.push_back(pose.xrot);
    e.push_back(pose.yrot);
    e.push_back(pose.zrot);

    if (angleUnits_ == DEGREE)
    {
      e.at(0) *= (3.141592654f / 180.0f);
      e.at(1) *= (3.141592654f / 180.0f);
      e.at(2) *= (3.141592654f / 180.0f);
    }

    m1.rotEulerMatrixConvert(e);
    m1.rotMatrixQuaternionConvert(q);

    target.push_back (q.at(0));
    target.push_back (q.at(1));
    target.push_back (q.at(2));
    target.push_back (q.at(3));

    //! PTP, Cartesian, Absolute
    //if (generateMove ('P', 'C', 'R', target)) //! JAM:  for initial testing purposes only
    ulapi_mutex_take(ka_.handle);
    if (generateMove ('P', 'C', 'A', target))
    {
      //! Send message to robot
      if (!send())
      {
        //! error sending
        printf("failed send\n");
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      //! Wait for response from robot
      if (!get ())
      {
        printf("failed get\n");
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }
      ulapi_mutex_give(ka_.handle);
      if (mssgBuffer_[1] == '1')
      {
        //printf("got message\n");
        return CANON_SUCCESS;
      }
      else
      {
        //printf("bad return\n");
        return CANON_FAILURE;
      }
    }
    else
    {
      //! Error generating motion message
      printf("bad message\n");
      ulapi_mutex_give(ka_.handle);
      return CANON_FAILURE;
    }
  }


  LIBRARY_API CanonReturn CrpiAbb::GetRobotAxes (robotAxes *axes)
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
      if (!parseFeedback (8))
      {
        //! Error parsing data
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }

      ulapi_mutex_give(ka_.handle);
      try
      {
        for (int i = 0; i < 7; ++i)
        {
          axes->axis.at(i) = feedback_[i+1];
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


  LIBRARY_API CanonReturn CrpiAbb::GetRobotForces (robotPose *forces)
  {
    //! Not supported by the ABB Rapid interface
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiAbb::GetRobotIO (robotIO *io)
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


  LIBRARY_API CanonReturn CrpiAbb::GetRobotPose (robotPose *pose)
  {
    double qx, qy, qz, qw;
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
        pose->x = feedback_[1];
        pose->y = feedback_[2];
        pose->z = feedback_[3];
        qw = feedback_[4]; //qx
        qx = feedback_[5]; //qy
        qy = feedback_[6]; //qz
        qz = feedback_[7]; //qw

        Math::matrix m1(3,3);
        vector<double> q, e;
        q.push_back(qw);
        q.push_back(qx);
        q.push_back(qy);
        q.push_back(qz);

        m1.rotQuaternionMatrixConvert(q);
        m1.rotMatrixEulerConvert(e);

        pose->xrot = e.at(0);
        pose->yrot = e.at(1);
        pose->zrot = e.at(2);

        pose->status = 0;
        pose->turns = 0;

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

        if (angleUnits_ == DEGREE)
        {
          pose->zrot *= (180.0f / 3.141592654f);
          pose->yrot *= (180.0f / 3.141592654f);
          pose->xrot *= (180.0f / 3.141592654f);
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


  LIBRARY_API CanonReturn CrpiAbb::GetRobotSpeed (robotPose *speed)
  {
    //! TODO
    return CANON_FAILURE;
  }


  LIBRARY_API CanonReturn CrpiAbb::GetRobotSpeed (robotAxes *speed)
  {
    //! TODO
    return CANON_FAILURE;
  }


  LIBRARY_API CanonReturn CrpiAbb::GetRobotTorques (robotAxes *torques)
  {
    //! Construct request for axis torque information

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
      if (!parseFeedback(8))
      {
        //! Error parsing data
        ulapi_mutex_give(ka_.handle);
        return CANON_FAILURE;
      }

      ulapi_mutex_give(ka_.handle);
      try
      {
        for (int i = 0; i < 7; ++i)
        {
          torques->axis.at(i) = feedback_[i + 1];
          if (angleUnits_ == RADIAN)
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
    } // if generateFeedback ('A')
    else
    {
      //! Error generating feedback request message
      ulapi_mutex_give(ka_.handle);
      return CANON_FAILURE;
    }
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiAbb::MoveAttractor (robotPose &pose)
  {
    //! Not supported
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiAbb::MoveToAxisTarget (robotAxes &axes)
  {
    //! Construct message
    vector<double> target;
    for (int i = 0; i < 7; ++i)
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


  LIBRARY_API CanonReturn CrpiAbb::SetAbsoluteAcceleration (double tolerance)
  {
    //! Not yet implemented
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiAbb::SetAbsoluteSpeed (double speed)
  {
    double scale = 1.0f, newspeed = 0.0f;
    if (lengthUnits_ == METER)
    {
      scale = 1000.0f;
    }
    else if (lengthUnits_ == INCH)
    {
      scale = 25.4f;
    }
    
    newspeed = speed * scale;
    newspeed = (newspeed <= 1000.0f ? newspeed : 1000.0f);
    return SetRelativeSpeed(newspeed / 1000.0f);
  }

//  ("degree" or "radian")
  LIBRARY_API CanonReturn CrpiAbb::SetAngleUnits (const char *unitName)
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


  LIBRARY_API CanonReturn CrpiAbb::SetAxialSpeeds (double *speeds)
  {
    //! Not yet implemented
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiAbb::SetAxialUnits (const char **unitNames)
  {
    //! Not yet implemented
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiAbb::SetEndPoseTolerance (robotPose &tolerances)
  {
    //! Not yet implemented
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiAbb::SetIntermediatePoseTolerance (robotPose *tolerances)
  {
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiAbb::SetLengthUnits (const char *unitName)
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


  LIBRARY_API CanonReturn CrpiAbb::SetParameter (const char *paramName, void *paramVal)
  {
    //! Not yet implemented
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiAbb::SetRelativeAcceleration (double percent)
  {
    //! Not yet implemented
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiAbb::SetRelativeSpeed (double percent)
  {
    double sendme = 5.0f;
    vector<double> param;
    param.clear();
    if (percent < 0.005f)
    {
      sendme = 0.005f;
    }
    else if (percent > 1.0f)
    {
      sendme = 1.0f;
    }
    param.push_back(sendme*1000.0f);

    generateParameter('S', 'R', param);
    if (send())
    {
      //! TODO
    }

    //! Not yet implemented
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiAbb::SetRobotIO (robotIO &io)
  {
    bool state = true;
    for (int x = 0; x < 7; ++x)
    {
      state &= (SetRobotDO(x, io.dio[x]) == CANON_SUCCESS);
    }

    return (state ? CANON_SUCCESS : CANON_FAILURE);
  }


  LIBRARY_API CanonReturn CrpiAbb::SetRobotDO (int dig_out, bool val)
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


  LIBRARY_API CanonReturn CrpiAbb::StopMotion (int condition)
  {
    //! Not yet implemented
    return CANON_REJECT;
  }
  

  LIBRARY_API bool CrpiAbb::generateMove (char moveType, char posType, char deltaType, vector<double> &input)
  {
    int cmdNum = 0;
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
    state &= (posType == 'C' || posType == 'A');
    //!   Check absolute or relative motion
    state &= (deltaType == 'A' || deltaType == 'R');

    if (!state)
    {
      printf("bad args\n");
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

    cmdNum = ((moveType == 'P') ? 200 : 300) + ((posType == 'C') ? 0 : 10) + ((deltaType == 'A') ? 0 : 1);
    moveMe_ << "[" << cmdNum << ",";

    //! Loop through paramaters, completes 10 char string, and adds to command string
    for (i = 0; i < 7 ; ++i)
    {
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

      for (j = currLength; j < 9; ++j)
      {
        tempString_ << "0";
      }

      moveMe_ << tempString_.str();
      if (i < 6)
      {
        moveMe_ << ",";
      }
    } // for (i = 0; i < 7; ++i)
    moveMe_ << "]\0";

    return true;
  }


  LIBRARY_API bool CrpiAbb::generateTool (char mode, double value)
  {
    if (!(mode == 'B' || mode == 'A'))
    {
      return false;
    }
    size_t found;
    int currLength, j;
    int cmd = 0;

    cmd = ((mode == 'B') ? 0 : ((mode == 'A') ? 10 : 20));
    //printf ("Tool: %d\n", cmd);
    moveMe_.str(string());

    moveMe_ << "[" << cmd << ",";

    //! Convert the input value to a character string of length 10
    tempString_.str(string());
    tempString_ << (1.0 - value);

    //! If number is an integer, add a decimal point
    found = tempString_.str().find('.');
    if (found == string::npos)
    {
      tempString_ << ".";
    }

    //! Add trailing 0s to decimal to create 10-char number string
    tempString_.seekg (0, ios::end);
    currLength = (int) tempString_.tellg ();

    for (j = currLength; j < 9; ++j)
    {
      tempString_ << "0";
    }

    moveMe_ << tempString_.str() << ",";
    moveMe_ << "0.0000000,0.0000000,0.0000000,0.0000000,0.0000000,0.0000000]\0";

#ifdef ABB_NOISY
    printf(moveMe_.str().c_str());
#endif
    return true;
  }


  LIBRARY_API bool CrpiAbb::generateFeedback (char retType)
  {
    if (retType != 'C' && retType != 'A' && retType != 'F' && retType != 'T' && retType != 'S')
    {
      //! Unsupported variable
      return false;
    }
    int cmd = 0;

    switch (retType)
    {
    case 'C':
      cmd = 500;
      break;
    case 'A':
      cmd = 600;
      break;
    case 'F':
      cmd = 700;
      break;
    case 'T':
      cmd = 800;
      break;
    case 'S':
      cmd = 900;
      break;
    default:
      break;
    }

    moveMe_.str(string());
    moveMe_ << "[" << cmd << ",0.0000000,0.0000000,0.0000000,0.0000000,0.0000000,0.0000000,0.0000000]\0";
    
    return true;
  }


  LIBRARY_API bool CrpiAbb::generateIO(char sigtype, int signum, double val )
  {
    if (sigtype != 'D' && sigtype != 'A')
    {
      //! Unsupported variable
      return false;
    }
    int cmd = 0;

    switch (sigtype)
    {
    case 'D':
      cmd = 400;
      break;
    case 'A':
      cmd = 410;
      break;
    default:
      break;
    }

    moveMe_.str(string());
    moveMe_ << "[" << cmd << "," << signum << "," << val << ",0.0000000,0.0000000,0.0000000,0.0000000,0.0000000]\0";

    return true;
  }

  LIBRARY_API bool CrpiAbb::generateParameter (char paramType, char subType, vector<double> &input)
  {
    size_t found;
    int j = 0, currLength = 0;
    int cmd = 0;
    
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

      if (subType == 'R')
      {
        //! Relative
        if (input[0] > 1000.0f || input[0] < 0.0f)
        {
          return false;
        }
      }
      else
      {
        return false;
      }

      cmd = 100 + ((paramType == 'S') ? 0 : 10);

      moveMe_.str (string());
      moveMe_ << "[" << cmd << ",";

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

      for (j = currLength; j < 9; ++j)
      {
        tempString_ << "0";
      }

      moveMe_ << tempString_.str() << ",0.0000000,0.0000000,0.0000000,0.0000000,0.0000000,0.0000000]\0";

      break;
    default:
      return false;
    }
    
    return true;
  }

  
  LIBRARY_API bool CrpiAbb::send ()
  {
    int x;
    
#ifdef ABB_NOISY
    printf ("server_ = %d\n", server_);
    printf ("Sending message %s\n", moveMe_.str().c_str());
#endif
      //! Use TCP/IP
      x = ulapi_socket_write(server_, moveMe_.str().c_str(), strlen(moveMe_.str().c_str())+1);
#ifdef ABB_NOISY
      printf ("%i\n", x);
#endif
      return true;
  }


  LIBRARY_API bool CrpiAbb::get ()
  {
    int x = 0;

#ifdef ABB_NOISY
      printf ("getting feedback...\n");
#endif

      //! Use TCP/IP
      x = ulapi_socket_read(server_, mssgBuffer_, 8192);
#ifdef ABB_NOISY
      printf ("%d %s read\n", x, mssgBuffer_);
#endif
      return true;
  }


  LIBRARY_API bool CrpiAbb::parseFeedback (int num)
  {
    bool newItem = false;
    int index = -1, i;

    for (i = 0; i < num; ++i)
    {
      tempData_->at(i) = "";
    }

    //! REQUEST_MSG_SIZE is very large, feedback from robot is limited to 80 characters
    for (i = 1; i < 80; ++i)
    {
      if (mssgBuffer_[i] == ',' || mssgBuffer_[i] == ']')
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
