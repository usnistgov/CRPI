///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Robot Interface
//  Workfile:        CrpiSchunkSDH.cpp
//  Revision:        1.0 - 13 March, 2014
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Schunk SDH interface definitions.
//
///////////////////////////////////////////////////////////////////////////////

#include "crpi_schunk_sdh.h"
#include <fstream>
#include <iostream>
#include <string.h>
#include <sstream>

using namespace std;

namespace crpi_robot
{

  LIBRARY_API CrpiSchunkSDH::CrpiSchunkSDH (CrpiRobotParams &params)
  {
    server = ulapi_socket_get_client_id (6009, "127.0.0.1");

    if (server < 0)
    {
      cout << "no connection" << endl;
    }
    else
    {
      cout << "connection success" << endl;
    }
  }

  LIBRARY_API CrpiSchunkSDH::~CrpiSchunkSDH ()
  {
  }

  LIBRARY_API CanonReturn CrpiSchunkSDH::ApplyCartesianForceTorque (robotPose &robotForceTorque, vector<bool> activeAxes, vector<bool> manipulator)
  {
    //! Not supported
    return CANON_REJECT;
  }

  LIBRARY_API CanonReturn CrpiSchunkSDH::ApplyJointTorque (robotAxes &robotJointTorque)
  {
    //! TODO
    return CANON_FAILURE;
  }

  LIBRARY_API CanonReturn CrpiSchunkSDH::SetTool (double percent)
  {
    strcpy(outbuffer,"");
    strcpy(inbuffer,"");

    if (percent == 1) 
    {
      strcpy(outbuffer,"Grasp");
      send = ulapi_socket_write(server, outbuffer, sizeof(outbuffer));
      Sleep(50);
    }
    else if (percent == 0) 
    {
      strcpy(outbuffer,"Pause");
      send = ulapi_socket_write(server, outbuffer, sizeof(outbuffer));
      Sleep(50);
    }
    else if (percent == -1) 
    {
      strcpy(outbuffer,"Stop");
      send = ulapi_socket_write(server, outbuffer, sizeof(outbuffer));
      Sleep(50);
      //get = ulapi_socket_read(server, inbuffer, MSG_SIZE);
      //cout << inbuffer << endl;
    }
    else
    {
      return CANON_FAILURE;
    }

    strcpy(outbuffer,"");
    strcpy(inbuffer,"");
    
    return CANON_SUCCESS;
  }

  LIBRARY_API CanonReturn CrpiSchunkSDH::Couple (const char *targetID)
  {
    //! Not supported
    return CANON_REJECT;
  }

  LIBRARY_API CanonReturn CrpiSchunkSDH::Message (const char *message)
  {
    //! Not supported
    return CANON_REJECT;
  }

  LIBRARY_API CanonReturn CrpiSchunkSDH::MoveStraightTo (robotPose &pose)
  {
    //! Not supported
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiSchunkSDH::MoveThroughTo (robotPose *poses,
                                                        int numPoses,
                                                        robotPose *accelerations,
                                                        robotPose *speeds,
                                                        robotPose *tolerances)
  {
    //! Not supported
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiSchunkSDH::MoveTo (robotPose &pose)
  {
    //! Not supported
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiSchunkSDH::GetRobotAxes (robotAxes *axes)
  {
    //! TODO
    return CANON_FAILURE;
  }

 
  LIBRARY_API CanonReturn CrpiSchunkSDH::GetRobotForces (robotPose *forces)
  {
    //! TODO
    return CANON_FAILURE;
  }


  LIBRARY_API CanonReturn CrpiSchunkSDH::GetRobotIO (robotIO *io)
  {
  //readSDH();

  strcpy(inbuffer,"");

  get = ulapi_socket_read(server, inbuffer, MSG_SIZE);
  
  if (strcmp(inbuffer,"Success")==0)
  {
    strcpy(inbuffer,"");
    return CANON_SUCCESS;
  }

  else {return CANON_FAILURE;}
  }


  LIBRARY_API CanonReturn CrpiSchunkSDH::GetRobotPose (robotPose *pose)
  {
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiSchunkSDH::GetRobotSpeed (robotPose *speed)
  {
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiSchunkSDH::GetRobotSpeed (robotAxes *speed)
  {
    //! TODO
    return CANON_FAILURE;
  }


  LIBRARY_API CanonReturn CrpiSchunkSDH::GetRobotTorques (robotAxes *torques)
  {
    //! TODO
    return CANON_FAILURE;
  }


  LIBRARY_API CanonReturn CrpiSchunkSDH::MoveAttractor (robotPose &pose)
  {
    //! Not supported
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiSchunkSDH::MoveToAxisTarget (robotAxes &axes)
  {
    std::ostringstream sstream;
    std::string AngleAsString;

    strcpy(outbuffer,"");

    for (unsigned int i=0;i<7;++i)
    {
      sstream << axes.axis.at(i);
      AngleAsString = sstream.str();
      strcat(outbuffer, AngleAsString.c_str());
      strcat(outbuffer, " ");
      sstream.str(std::string());
    }

    send = ulapi_socket_write(server, outbuffer, sizeof(outbuffer));

    Sleep(50);
    
    strcpy(outbuffer,"");

    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiSchunkSDH::SetAbsoluteAcceleration (double tolerance)
  {
    //! TODO
    return CANON_FAILURE;
  }


  LIBRARY_API CanonReturn CrpiSchunkSDH::SetAbsoluteSpeed (double speed)
  {
    //! TODO
    return CANON_FAILURE;
  }


  LIBRARY_API CanonReturn CrpiSchunkSDH::SetAngleUnits (const char *unitName)
  {
    //! TODO
    return CANON_FAILURE;
  }


  LIBRARY_API CanonReturn CrpiSchunkSDH::SetAxialSpeeds (double *speeds)
  {
    //! TODO
    return CANON_FAILURE;
  }


  LIBRARY_API CanonReturn CrpiSchunkSDH::SetAxialUnits (const char **unitNames)
  {
    //! TODO
    return CANON_FAILURE;
  }


  LIBRARY_API CanonReturn CrpiSchunkSDH::SetEndPoseTolerance (robotPose &tolerances)
  {
    //! Not supported
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiSchunkSDH::SetIntermediatePoseTolerance (robotPose *tolerances)
  {
    //! Not supported
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiSchunkSDH::SetLengthUnits (const char *unitName)
  {
    //! Not supported
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiSchunkSDH::SetParameter (const char *paramName, void *paramVal)
  {
    int *temp_int = (int*) paramVal;
    int val = *temp_int;

    strcpy(outbuffer,"");

    if ((strcmp (paramName, "GRIP_TYPE") == 0))
    {
      strcpy(outbuffer, paramName);
      strcat(outbuffer, ",");
      if (val == 1)
        strcat(outbuffer, "1");
      else if(val == 2)
        strcat(outbuffer, "2");
      else if(val == 3)
        strcat(outbuffer, "3");

      send = ulapi_socket_write(server, outbuffer, sizeof(outbuffer));

      Sleep(50);

      get = ulapi_socket_read(server, inbuffer, MSG_SIZE);

      cout << inbuffer << endl;
    }

  if ((strcmp (paramName, "NUM_FINGERS") == 0))
  {
    if (val == 1) {strcpy(outbuffer, "1");}
    else if (val == 2) {strcpy(outbuffer, "2");}
    else if (val == 3) {strcpy(outbuffer, "3");}
    else if (val == 0) {strcpy(outbuffer, "0");} //no closing fingers

    send = ulapi_socket_write(server, outbuffer, sizeof(outbuffer));
    Sleep(50);
  }

    strcpy(outbuffer,"");
    strcpy(inbuffer,"");

    return CANON_SUCCESS;
  }

  LIBRARY_API CanonReturn CrpiSchunkSDH::SetRelativeAcceleration (double percent)
  {
    //! TODO
    return CANON_FAILURE;
  }


  LIBRARY_API CanonReturn CrpiSchunkSDH::SetRelativeSpeed (double percent)
  {
    //! TODO
    return CANON_FAILURE;
  }


  LIBRARY_API CanonReturn CrpiSchunkSDH::SetRobotIO (robotIO &io)
  {
    //! Not supported
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiSchunkSDH::SetRobotDO (int dig_out, bool val)
  {
    //! Not supported
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiSchunkSDH::StopMotion (int condition)
  {
    //! TODO
    return CANON_FAILURE;
  }

} // crpi_robot
