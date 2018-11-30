///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Robot Interface
//  Workfile:        crpi_allegro.cpp
//  Revision:        1.0 - 5 November, 2015
//  Author:          K. Van Wyk and J. Marvel
//
//  Description
//  ===========
//  Allegro Hand interface definitions.
//
///////////////////////////////////////////////////////////////////////////////

#include "crpi_allegro.h"
#include <fstream>
#include <iostream>
#include <string.h>
#include <sstream>


using namespace std;

namespace crpi_robot
{

  LIBRARY_API CrpiAllegro::CrpiAllegro (CrpiRobotParams &params)
  {
    server_config = ulapi_socket_get_client_id (6008, "127.0.0.1");

    server_params = ulapi_socket_get_client_id (6009, "127.0.0.1");

    server_feedback = ulapi_socket_get_client_id (6011, "127.0.0.1");

    if (server_config < 0)
    {
      cout << "no connection for controllers" << endl;
    }
    else
    {
      cout << "connection success for controllers" << endl;
    }

    if (server_params < 0)
    {
      cout << "no connection for parameters" << endl;
    }
    else
    {
      cout << "connection success for parameters" << endl;
    }

    if (server_feedback < 0)
    {
      cout << "no connection for feedback" << endl;
    }
    else
    {
      cout << "connection success for feedback" << endl;
    }
  }

  LIBRARY_API CrpiAllegro::~CrpiAllegro ()
  {
  }

  LIBRARY_API CanonReturn CrpiAllegro::SetTool (double percent)
  {
    //This is probably a good place to start, pause, and stop control processes

    /*
    if (percent == 1) 
    {
      strcpy(outbuffer,"Grasp");
      send = ulapi_socket_write(server, outbuffer, sizeof(outbuffer));
      //get = ulapi_socket_read(server, inbuffer, MSG_SIZE);
      //cout << inbuffer << endl;
    }

    else if (percent == 0) 
    {
      strcpy(outbuffer,"Pause");
      send = ulapi_socket_write(server, outbuffer, sizeof(outbuffer));
      //get = ulapi_socket_read(server, inbuffer, MSG_SIZE);
      //cout << inbuffer << endl;
    }

    else if (percent == -1) 
    {
      strcpy(outbuffer,"Stop");
      send = ulapi_socket_write(server, outbuffer, sizeof(outbuffer));
      //get = ulapi_socket_read(server, inbuffer, MSG_SIZE);
      //cout << inbuffer << endl;
    }

    strcpy(outbuffer,"");
    strcpy(inbuffer,"");
    
    */
    return CANON_SUCCESS;
  }

  LIBRARY_API CanonReturn CrpiAllegro::ApplyCartesianForceTorque (robotPose &robotForceTorque, vector<bool> activeAxes, vector<bool> manipulator)
  {
    //This assumes that all toggled fingers will experience the same force/torque and active axes commands, unless disengaged with the manipulator command
    // FMP
    SetParameter("Control", const_cast<char *>("finger_force"));
    SetParameter("Plan", const_cast<char *>("set_point")); //will need to change this to file for finger trajectory tracking for greater command bandwidth
    
    std::ostringstream sstream;
    std::string ForcesAsString;
    double forces[12] = {0};

    strcpy(outbuffer,"");

    //populate force vector from inputs
    for (unsigned int ii=0; ii<manipulator.size(); ++ii)
    {
      if (manipulator[ii] == true)
      {
        forces[ii*3+0] = robotForceTorque.x;
        forces[ii*3+1] = robotForceTorque.y;
        forces[ii*3+2] = robotForceTorque.z;
      }
    }

    //Convert force vector to space-separated, char array
    for(unsigned int i=0;i<12;++i)
    {
      sstream << forces[i];
      ForcesAsString = sstream.str();
      strcat(outbuffer, ForcesAsString.c_str());
      strcat(outbuffer, " ");
      sstream.str(std::string());
    }

    send = ulapi_socket_write(server_config, outbuffer, sizeof(outbuffer));
    Sleep(3);

    strcpy(outbuffer,"");
    
    return CANON_SUCCESS;
  }

  LIBRARY_API CanonReturn CrpiAllegro::ApplyJointTorque (robotAxes &robotJointTorque)
  {
    return CANON_REJECT;
  }

  LIBRARY_API CanonReturn CrpiAllegro::Couple (const char *targetID)
  {
    return CANON_REJECT;
  }

  LIBRARY_API CanonReturn CrpiAllegro::Message (const char *message)
  {
    return CANON_REJECT;
  }

  LIBRARY_API CanonReturn CrpiAllegro::MoveStraightTo (robotPose &pose)
  {
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiAllegro::MoveThroughTo (robotPose *poses,
                                                      int numPoses,
                                                      robotPose *accelerations,
                                                      robotPose *speeds,
                                                      robotPose *tolerances)
  {
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiAllegro::MoveTo (robotPose &pose)
  {
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiAllegro::GetRobotAxes (robotAxes *axes)
  {
    strcpy(outbuffer,"");
    strcpy(outbuffer,"joint_angles");

    send = ulapi_socket_write(server_feedback, outbuffer, MSG_SIZE);
    Sleep(10);
    
    strcpy(inbuffer,"");
    get = ulapi_socket_read(server_feedback, inbuffer, MSG_SIZE);

    //Copy over into string and replace commas with spaces
    std::string temp(inbuffer);
    int data_size = temp.size();

    //Copy back over into existing char array
    for (int ii=0;ii<data_size; ++ii)
    {
      inbuffer[ii] = temp[ii];
    }

    //Pull out joint angles assuming they all exist and are in order, and are in a char array with spaces separating values
    char* pEnd;
  
    axes->axis.at(0) = strtod (inbuffer, &pEnd); //get first angle
    //std::cout << axes->axis.at(0) << " ";
  
    for (unsigned short int jj=1;jj<=14;++jj) //gets other angles
    {
      axes->axis.at(jj) = strtod (pEnd, &pEnd);
      //std::cout << axes->axis.at(jj) << " ";
    }
    axes->axis.at(15) = strtod (pEnd, NULL); //get last angle
    //std::cout << axes->axis.at(15) << " ";

    return CANON_SUCCESS;
  }

 
  LIBRARY_API CanonReturn CrpiAllegro::GetRobotForces (robotPose *forces)
  {

    strcpy(outbuffer,"");
    strcpy(outbuffer,"cart_force");

    send = ulapi_socket_write(server_feedback, outbuffer, MSG_SIZE);
    Sleep(10);
    
    strcpy(inbuffer,"");
    get = ulapi_socket_read(server_feedback, inbuffer, MSG_SIZE);

    //Copy over into string and replace commas with spaces
    std::string temp(inbuffer);
    int data_size = temp.size();

    //Copy back over into existing char array
    for (int ii=0;ii<data_size; ++ii)
    {
      inbuffer[ii] = temp[ii];
    }

    //Pull out joint angles assuming they all exist and are in order, and are in a char array with spaces separating values
    char* pEnd;
  
    forces->x = strtod (inbuffer, &pEnd);
    forces->y = strtod (pEnd, &pEnd);
    forces->z = strtod (pEnd, &pEnd);
    //no torques yet
    forces->xrot = 0;//strtod (pEnd, &pEnd);
    forces->yrot = 0;//strtod (pEnd, &pEnd);
    forces->zrot = 0;//strtod (pEnd, NULL);

    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiAllegro::GetRobotIO (robotIO *io)
  {
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiAllegro::GetRobotPose (robotPose *pose)
  {
    strcpy(outbuffer,"");
    strcpy(outbuffer,"cart_pose");

    send = ulapi_socket_write(server_feedback, outbuffer, MSG_SIZE);
    Sleep(10);
    
    strcpy(inbuffer,"");
    get = ulapi_socket_read(server_feedback, inbuffer, MSG_SIZE);

    //Copy over into string and replace commas with spaces
    std::string temp(inbuffer);
    int data_size = temp.size();

    //Copy back over into existing char array
    for (int ii=0;ii<data_size; ++ii)
    {
      inbuffer[ii] = temp[ii];
    }

    //Pull out joint angles assuming they all exist and are in order, and are in a char array with spaces separating values
    char* pEnd;
  
    pose->x = strtod (inbuffer, &pEnd); //get first angle
    pose->y = strtod (pEnd, &pEnd);
    pose->z = strtod (pEnd, &pEnd);
    pose->xrot = strtod (pEnd, &pEnd);
    pose->yrot = strtod (pEnd, &pEnd);
    pose->zrot = strtod (pEnd, NULL); //get last angle
    //std::cout << axes->axis.at(15) << " ";

    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiAllegro::GetRobotSpeed (robotPose *speed)
  {
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiAllegro::GetRobotSpeed (robotAxes *speed)
  {
    //! TODO
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiAllegro::GetRobotTorques (robotAxes *torques)
  {
    //! TODO
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiAllegro::MoveAttractor (robotPose &pose) //hijacked for grasping control
  {
    // FMP
    SetParameter("Control", const_cast<char *>("grasping"));
    SetParameter("Plan", const_cast<char *>("set_point"));
    
    std::ostringstream sstream;
    std::string PoseAsString;

    strcpy(outbuffer,"");

    sstream << pose.x;
    PoseAsString = sstream.str();
    strcat(outbuffer, PoseAsString.c_str());
    strcat(outbuffer, " ");
    sstream.str(std::string());

    sstream << pose.y;
    PoseAsString = sstream.str();
    strcat(outbuffer, PoseAsString.c_str());
    strcat(outbuffer, " ");
    sstream.str(std::string());

    sstream << pose.z;
    PoseAsString = sstream.str();
    strcat(outbuffer, PoseAsString.c_str());
    strcat(outbuffer, " ");
    sstream.str(std::string());

    sstream << pose.xrot;
    PoseAsString = sstream.str();
    strcat(outbuffer, PoseAsString.c_str());
    strcat(outbuffer, " ");
    sstream.str(std::string());

    sstream << pose.yrot;
    PoseAsString = sstream.str();
    strcat(outbuffer, PoseAsString.c_str());
    strcat(outbuffer, " ");
    sstream.str(std::string());

    sstream << pose.zrot;
    PoseAsString = sstream.str();
    strcat(outbuffer, PoseAsString.c_str());
    strcat(outbuffer, " ");
    sstream.str(std::string());

    send = ulapi_socket_write(server_config, outbuffer, sizeof(outbuffer));
    Sleep(10);

    strcpy(outbuffer,"");
    
    return CANON_SUCCESS;

  }


  LIBRARY_API CanonReturn CrpiAllegro::MoveToAxisTarget (robotAxes &axes)
  {
    // FMP
    SetParameter("Control", const_cast<char *>("joint_pos"));
    SetParameter("Plan", const_cast<char *>("set_point_smooth"));
    
    std::ostringstream sstream;
    std::string AngleAsString;

    strcpy(outbuffer,"");

    for (unsigned int i=0;i<16;++i)
    {
      sstream << axes.axis.at(i);
      AngleAsString = sstream.str();
      strcat(outbuffer, AngleAsString.c_str());
      strcat(outbuffer, " ");
      sstream.str(std::string());
    }

    send = ulapi_socket_write(server_config, outbuffer, sizeof(outbuffer));
    Sleep(10);

    strcpy(outbuffer,"");
    
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiAllegro::SetAbsoluteAcceleration (double tolerance)
  {
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiAllegro::SetAbsoluteSpeed (double speed)
  {
    std::ostringstream sstream;
    std::string SpeedAsString;

    strcpy(outbuffer,"");
    strcat(outbuffer, "Speed");
    send = ulapi_socket_write(server_params, outbuffer, sizeof(outbuffer));
    Sleep(10);
    strcpy(outbuffer,"");
  
    sstream << speed;
    SpeedAsString = sstream.str();
    strcat(outbuffer, SpeedAsString.c_str());
    send = ulapi_socket_write(server_params, outbuffer, sizeof(outbuffer));
    Sleep(10);
    strcpy(outbuffer,"");

    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiAllegro::SetAngleUnits (const char *unitName)
  {
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiAllegro::SetAxialSpeeds (double *speeds)
  {
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiAllegro::SetAxialUnits (const char **unitNames)
  {
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiAllegro::SetEndPoseTolerance (robotPose &tolerances)
  {
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiAllegro::SetIntermediatePoseTolerance (robotPose *tolerances)
  {
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiAllegro::SetLengthUnits (const char *unitName)
  {
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiAllegro::SetParameter (const char *paramName, void *paramVal)
  {
    //paramName is control type, control parameters, or control trajectory source
    //paramVal is the value

    //control modes: read_only, joint_pos, gravity_comp
    //planner  modes: set_point, set_point_smooth, function, file, feed_forward

    char * temp_char = (char*) paramVal;
    //int val = *temp_int;
  
    strcpy(outbuffer,"");

    if (strcmp(paramName,"Control")==0 || strcmp(paramName,"Plan")==0)
    {
      strcat(outbuffer, temp_char);
      send = ulapi_socket_write(server_config, outbuffer, sizeof(outbuffer));
      Sleep(10);
    }

    //parameters for various other behaviors:
    //touch_stop - stops motion of fingers once the sensor feels touch

    else if  (strcmp(paramName,"touch_stop")==0)
    {
      strcat(outbuffer, "touch_stop");
      send = ulapi_socket_write(server_params, outbuffer, sizeof(outbuffer));
      Sleep(10);
      strcpy(outbuffer,"");

      strcat(outbuffer, temp_char);
      send = ulapi_socket_write(server_params, outbuffer, sizeof(outbuffer));
      Sleep(10);
    }

    else if  (strcmp(paramName,"gravity_vector")==0)
    {
      strcat(outbuffer, "gravity_vector");
      send = ulapi_socket_write(server_params, outbuffer, sizeof(outbuffer));
      Sleep(10);
      strcpy(outbuffer,"");

      strcat(outbuffer, temp_char);
      send = ulapi_socket_write(server_params, outbuffer, sizeof(outbuffer));
      Sleep(10);
    }

    else if  (strcmp(paramName,"tare_nano17")==0)
    {
      cout << "TARING SENSORS" << endl;
      strcat(outbuffer, "tare_nano17");
      send = ulapi_socket_write(server_params, outbuffer, sizeof(outbuffer));
      Sleep(10);
    }

    /*
    else
    {
      strcat(outbuffer, "forces.csv");
      send = ulapi_socket_write(server_config, outbuffer, sizeof(outbuffer));
      Sleep(10);
    }
    */

    strcpy(outbuffer,"");

    return CANON_SUCCESS;

  }

  LIBRARY_API CanonReturn CrpiAllegro::SetRelativeAcceleration (double percent)
  {
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiAllegro::SetRelativeSpeed (double percent)
  {
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiAllegro::SetRobotIO (robotIO &io)
  {
    
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiAllegro::SetRobotDO (int dig_out, bool val)
  {
    //! TODO
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiAllegro::StopMotion (int condition)
  {
    return CANON_REJECT;
  }
  
} // crpi_robot
