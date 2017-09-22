///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Robot Interface
//  Workfile:        crpi_xml.cpp
//  Revision:        1.0 - 20 February, 2015
//
//  Author:          J. Marvel
//
//  Description
//  ===========
//  XML parser class definition file.
//
///////////////////////////////////////////////////////////////////////////////

#include "crpi_xml.h"
#include <iostream>

using namespace std;

namespace Xml
{
  LIBRARY_API CrpiXml::CrpiXml (CrpiXmlParams *params) :
    params_(params)
  {
    //xaxisactive = zaxisactive = false;
  }


  LIBRARY_API CrpiXml::~CrpiXml ()
  {
    params_ = NULL;
  }


  LIBRARY_API bool CrpiXml::characters(const vector<string>& ch)
  {
    vector<string>::const_iterator chIter = ch.begin();

    for (; chIter != ch.end(); ++chIter)
    {
    }

    return true;
  }


  LIBRARY_API bool CrpiXml::parse (const string &line)
  {
    static char buff[1024];
    bool readTags = false;
    bool state = true;
    bool error = false;

    xmlAttributes attr;
    vector<string> chars;

    int marker = 0;
    int openTags = 0;
    bool tagActive = false;
    bool start = false;
    bool single = false;
    bool validXML = false;

    int strLength = line.length();
    int count;
    string temp;
    string name;
    string attrname;
    string attrval;
    try
    {
      //! JAM:  TODO, STRIP OFF WHITE SPACE:  Carriage returns, tabs, etc.

      while (marker < strLength && !error && line[marker] != '\0')
      {
        if (line[marker] == '<')
        {
          //! ***** Handle tags *****
          tagActive = true;
          for (int i = 0; i < 20; i++)
          {
            buff[i] = ' ';
          }

          name = "";
          ++marker;

          //! Skip white space
          while (line[marker] == ' ' || line[marker] == '\t' || line[marker] == '?')
          {
            ++marker;
            if (marker >= (strLength - 1))
            {
              break;
            }
          }

          if (marker >= (strLength - 1))
          {
            //! Ill-formed XML string (end encountered before closing tags
            error = true;
            break;
          }

          if (line[marker] == '/')
          {
            start = false;
            marker++;
          }
          else
          {
            start = true;
          }

          int count = 0;
          while (line[marker] != ' ' && line[marker] != '\t' &&
                 line[marker] != '/' && line[marker] != '>' &&
                 line[marker] != '?' && marker <= (strLength - 1))
          {
            buff[count] = line[marker];
            count++;
            marker++;
            if (marker >= (strLength - 1))
            {
              break;
            }
          }
          if (marker > (strLength - 1))
          {
            //! Ill-formed XML string (end encountered before closing tags
            error = true;
            break;
          }

          buff[count] = '\0';
          name = buff;
          name = name.substr(0, count);

          if (start)
          {
            //! Look for attributes of opening tags
            while (line[marker] != '/' && line[marker] != '>' &&
                   line[marker] != '?' && marker <= (strLength -1))
            {
              if (line[marker] != ' ' && line[marker] != '\t' &&
                  line[marker] != '?' && line[marker] != '\n')
              {
                //! Non-white-space character & non-terminating element
                //! Ergo, an attribute
                count = 0;

                //! grab the attribute name
                while (line[marker] != '=' && line[marker] != ' ' && line[marker] != '?')
                {
                  buff[count++] = line[marker++];
                  if (marker >= strLength - 1)
                  {
                    break;
                  }
                }
                if (marker >= strLength - 1)
                {
                  break;
                }
                buff[count] = '\0';
                attrname = buff;
                attrname = attrname.substr(0, count);
                attr.name.push_back(attrname);

                //! grab the attribute value
                while (line[marker] != '\"')
                {
                  marker++;
                  if (marker >= strLength - 1)
                  {
                    break;
                  }
                }
                marker++;
                if (marker >= strLength - 1)
                {
                  break;
                }

                count = 0;
                while (line[marker] != '\"')
                {
                  buff[count++] = line[marker++];
                  if (marker >= strLength - 1)
                  {
                    break;
                  }
                }
                buff[count] = '\0';
                attrval = buff;
                attrval = attrval.substr(0, count);
                attr.val.push_back(attrval);
              } // if (line[marker] != ' ' && line[marker] != '\t' && line[marker] != '\n' && line[marker] != '?')
              marker++;
              if (marker > strLength - 1)
              {
                break;
              }
            } // while (line[marker] != '/' && line[marker] != '>' && line[marker] != '?')

            if (marker > (strLength - 1))
            {
              //! Ill-formed XML string (end encountered before closing tags
              error = true;
              break;
            } // if (marker > (strLength - 1))

            //! Test for single-tag XML lines (ex. <tag/>
            if (line[marker] == '/')
            {
              single = true;
            }
            while (line[marker] != '>')
            {
              marker++;
              if (marker >= strLength - 1)
              {
                break;
              }
            } // while (line[marker] != '>')
          } // if (start)

          if (error)
          {
            break;
          }

          //! Pass tag information for data structure assignment
          if (start)
          {
            if (state && startElement (name, attr))
            {
              state = true;
              attr.name.clear();
              attr.val.clear();
            }
            else
            {
              state = false;
            }

            validXML = false;
            openTags++;
            readTags = true;
            if (single)
            {
              tagActive = false;
              validXML = true;
              if (state && endElement (name))
              {
                state = true;
              }
              else
              {
                state = false;
              }
              openTags--;
              start = false;
            }
          } // if (start)
          else
          {
            tagActive = false;
            validXML = true;
            if (state && endElement (name))
            {
              state = true;
            }
            else
            {
              state = false;
            }
            openTags--;
          } // if (start) ... else
        } // if (line[marker] == '<')
        else if (tagActive)
        {
          //! ***** Handle text between tags *****
          while(start)
          {
            if (marker > strLength || line[marker] == '<')
            {
              marker--;
              break;
            }

            //! Skip white space
            while (line[marker] == ' ' || line[marker] == '\t' ||
                   line[marker] == '\n' || line[marker] == ',' ||
                   line[marker] == '?')
            {
              marker++;
              if (marker >= strLength)
              {
                break;
              }
            }

            if (marker > strLength || line[marker] == '<')
            {
              marker--;
              break;
            }

            count = 0;
            while (line[marker] != ',' && line[marker] != '<')
            {
              buff[count++] = line[marker++];
              if (marker >= strLength - 1)
              {
                break;
              }
            }
            if (marker >= (strLength - 1))
            {
              //! Ill-formed XML string (end encountered before closing tags
              error = true;
              break;
            }
            if (count > 0)
            {
              temp = buff;
              temp = temp.substr (0, count);
              chars.push_back (temp);
            }
          } // while(start)
          if (start)
          {
            if (state && interTagElement (name, chars))
            {
              state = true;
            }
            else
            {
              state = false;
            }
            chars.clear ();
          }
          start = false;
        } // else if (tagActive)

        ++marker;
        if (openTags < 1 && readTags)
        {
//          break;
        }
      } // while (marker < strLength && marker < 1024)
    } // try
    catch (...)
    {
      return false;
    }

    if (error)
    {
      //! Exiting with errors
    }
    return state;
  }

  /*
    Example commands for robot motion:

    <CRPICommand type="MoveTo">
      <Pose X="2.5" Y="1.0" Z="1.0" XRot="90.0" YRot="0.0" ZRot="0.0"/>
    </CRPICommand>

    <CRPICommand type="MoveToAxisTarget">
      <Axes J0="0.0" J1="0.0" J2="0.0" J3="0.0" J4="0.0" J5="0.0" J6="0.0"/>
    </CRPICommand>


    Example commands for tools:

    <CRPICommand type="SetTool">
      <Real Value="0.25"/>
    </CRPICommand>

    <CRPICommand type="Couple">
      <String Value="schunk_hand"/>
    </CRPICommand>


    <CRPICommand type="SetRobotDO">
      
      <Boolean Value="true"/>
    </CRPICommand>



    Example commands for robot configuration:

    <CRPICommand type="ToWorldMatrix">
      <Matrix4x4 V00="0.0" V01="0.0" V02="0.0" V03="0.0" V10="0.0" V11="0.0" V12="0.0" V13="0.0" V20="0.0" V21="0.0" V22="0.0" V23="0.0" V30="0.0" V31="0.0" V32="0.0" V33="0.0"/>
    </CRPICommand>


    Example commands for feedback:
    <CRPICommand type="GetRobotAxes">
    </CRPICommand>

    <CRPICommand type="GetRobotForces">
    </CRPICommand>
  */

  LIBRARY_API bool CrpiXml::startElement (const string& tagName, 
                                          const xmlAttributes& attr)
  {
    bool flag = true;
    std::vector<std::string>::const_iterator nameiter = attr.name.begin();
    std::vector<std::string>::const_iterator valiter = attr.val.begin();

    try
    {
      if (strcmp (tagName.c_str(), "CRPICommand") == 0)
      {
        for (; nameiter != attr.name.end(); ++nameiter, ++valiter)
        {
          if (strcmp (nameiter->c_str(), "type") == 0)
          {
            if (strcmp(valiter->c_str(), "ApplyCartesianForceTorque") == 0)
            {
              //CanonReturn ApplyCartesianForceTorque (robotPose robotForceTorque, vector<bool> activeAxes, vector<bool> manipulator);
              params_->cmd = CmdApplyCartesianForceTorque;
            }
            else if (strcmp(valiter->c_str(), "ApplyJointTorque") == 0)
            {
              //CanonReturn ApplyJointTorque(robotAxes robotJointTorque);
              params_->cmd = CmdApplyJointTorque;
            }
            else if (strcmp(valiter->c_str(), "Couple") == 0)
            {
              //CanonReturn Couple (const char *targetID);
              params_->cmd = CmdCouple;
            }
            else if (strcmp(valiter->c_str(), "GetRobotAxes") == 0)
            {
              //CanonReturn GetRobotAxes (robotAxes *axes);
              params_->cmd = CmdGetRobotAxes;
            }
            else if (strcmp(valiter->c_str(), "GetRobotForces") == 0)
            {
              //CanonReturn GetRobotForces (robotPose *forces);
              params_->cmd = CmdGetRobotForces;
            }
            else if (strcmp(valiter->c_str(), "GetRobotIO") == 0)
            {
              //CanonReturn GetRobotIO (robotIO *io);
              params_->cmd = CmdGetRobotIO;
            }
            else if (strcmp(valiter->c_str(), "GetRobotPose") == 0)
            {
              //CanonReturn GetRobotPose (robotPose *pose);
              params_->cmd = CmdGetRobotPose;
            }
            else if (strcmp(valiter->c_str(), "GetRobotSpeed") == 0)
            {
              //CanonReturn GetRobotSpeed(robotPose *speed);
              params_->cmd = CmdGetRobotSpeed;
            }
            else if (strcmp(valiter->c_str(), "GetRobotTorques") == 0)
            {
              //CanonReturn GetRobotTorques(robotAxes *torques);
              params_->cmd = CmdGetRobotTorques;
            }
            else if (strcmp(valiter->c_str(), "Message") == 0)
            {
              //CanonReturn Message(const char *message);
              params_->cmd = CmdMessage;
            }
            else if (strcmp(valiter->c_str(), "MoveAttractor") == 0)
            {
              //CanonReturn MoveAttractor(robotPose pose);
              params_->cmd = CmdMoveAttractor;
            }
            else if (strcmp(valiter->c_str(), "MoveStraightTo") == 0)
            {
              //CanonReturn MoveStraightTo(robotPose pose);
              params_->cmd = CmdMoveStraightTo;
            }
            else if (strcmp(valiter->c_str(), "MoveThroughTo") == 0)
            {
              //CanonReturn MoveThroughTo(robotPose *poses, int numPoses, robotPose *accelerations = NULL, robotPose *speeds = NULL, robotPose *tolerances = NULL);
              params_->cmd = CmdMoveThroughTo;
            }
            else if (strcmp(valiter->c_str(), "MoveTo") == 0)
            {
              //CanonReturn MoveTo(robotPose pose);
              params_->cmd = CmdMoveTo;
            }
            else if (strcmp(valiter->c_str(), "MoveToAxisTarget") == 0)
            {
              //CanonReturn MoveToAxisTarget(robotAxes axes);
             params_->cmd = CmdMoveToAxisTarget;
            }
            else if (strcmp(valiter->c_str(), "SetAbsoluteAcceleration") == 0)
            {
              //CanonReturn SetAbsoluteAcceleration (double acceleration);
              params_->cmd = CmdSetAbsoluteAcceleration;
            }
            else if (strcmp(valiter->c_str(), "SetAbsoluteSpeed") == 0)
            {
              //CanonReturn SetAbsoluteSpeed(double speed);
              params_->cmd = CmdSetAbsoluteSpeed;
            }
            else if (strcmp(valiter->c_str(), "SetAngleUnits") == 0)
            {
              //CanonReturn SetAngleUnits (const char *unitName);
              params_->cmd = CmdSetAngleUnits;
            }
            else if (strcmp(valiter->c_str(), "SetAxialSpeeds") == 0)
            {
              //CanonReturn SetAxialSpeeds(double *speeds);
              params_->cmd = CmdSetAxialSpeeds;
            }
            else if (strcmp(valiter->c_str(), "SetAxialUnits") == 0)
            {
              //CanonReturn SetAxialUnits(const char **unitNames);
              params_->cmd = CmdSetAxialUnits;
            }
            else if (strcmp(valiter->c_str(), "SetEndPoseTolerance") == 0)
            {
              //CanonReturn SetEndPoseTolerance(robotPose tolerance);
              params_->cmd = CmdSetEndPoseTolerance;
            }
            else if (strcmp(valiter->c_str(), "SetIntermediatePoseTolerance") == 0)
            {
              //CanonReturn SetIntermediatePoseTolerance(robotPose *tolerances);
              params_->cmd = CmdSetIntermediatePoseTolerance;
            }
            else if (strcmp(valiter->c_str(), "SetLengthUnits") == 0)
            {
              //CanonReturn SetLengthUnits(const char *unitName);
              params_->cmd = CmdSetLengthUnits;
            }
            else if (strcmp(valiter->c_str(), "SetParameter") == 0)
            {
              //CanonReturn SetParameter(const char *paramName, void *paramVal);
              params_->cmd = CmdSetParameter;
            }
            else if (strcmp(valiter->c_str(), "SetRelativeAcceleration") == 0)
            {
              //CanonReturn SetRelativeAcceleration(double percent);
              params_->cmd = CmdSetRelativeAcceleration;
            }
            else if (strcmp(valiter->c_str(), "SetRelativeSpeed") == 0)
            {
              //CanonReturn SetRelativeSpeed(double percent);
              params_->cmd = CmdSetRelativeSpeed;
            }
            else if (strcmp(valiter->c_str(), "SetRobotIO") == 0)
            {
              // CanonReturn SetRobotIO(robotIO io);
              params_->cmd = CmdSetRobotIO;
            }
            else if (strcmp(valiter->c_str(), "SetRobotDO") == 0)
            {
              //CanonReturn SetRobotDO(int dig_out, bool val);
              params_->cmd = CmdSetRobotDO;
            }
            else if (strcmp(valiter->c_str(), "SetTool") == 0)
            {
              //CanonReturn SetTool (double percent);
              params_->cmd = CmdSetTool;
            }
            else if (strcmp(valiter->c_str(), "StopMotion") == 0)
            {
              //CanonReturn StopMotion(int condition = 2);
              params_->cmd = CmdStopMotion;
            }
            else if (strcmp(valiter->c_str(), "ToWorldMatrix") == 0)
            {
              //CanonReturn ToWorldMatrix(matrix & R_T_W);
              params_->cmd = CmdToWorldMatrix;
            }
            else if (strcmp(valiter->c_str(), "ToSystemMatrix") == 0)
            {
              //CanonReturn ToCoordSystemMatrix(const char *name, matrix &R_T_S);
              params_->cmd = CmdToSystemMatrix;
            }
            else if (strcmp(valiter->c_str(), "ToWorld") == 0)
            {
              //CanonReturn ToWorld(robotPose *in, robotPose *out);
              params_->cmd = CmdToWorld;
            }
            else if (strcmp(valiter->c_str(), "FromWorld") == 0)
            {
              //CanonReturn FromWorld(robotPose *in, robotPose *out);
              params_->cmd = CmdFromWorld;
            }
            else if (strcmp(valiter->c_str(), "ToSystem") == 0)
            {
              //CanonReturn ToSystem(const char *name, robotPose *in, robotPose *out);
              params_->cmd = CmdToSystem;
            }
            else if (strcmp(valiter->c_str(), "FromSystem") == 0)
            {
              //CanonReturn FromSystem(const char *name, robotPose *in, robotPose *out);
              params_->cmd = CmdFromSystem;
            }
            else if (strcmp(valiter->c_str(), "UpdateWorldTransform") == 0)
            {
              //CanonReturn UpdateWorldTransform(robotPose &newToSystem);
              //CanonReturn UpdateWorldTransform(matrix &newToSystem);
              params_->cmd = CmdUpdateWorldTransform;
            }
            else if (strcmp(valiter->c_str(), "UpdateSystemTransform") == 0)
            {
              //CanonReturn UpdateSystemTransform(const char *name, robotPose &newToWorld);
              params_->cmd = CmdUpdateSystemTransform;
            }
            else if (strcmp(valiter->c_str(), "SaveConfig") == 0)
            {
              //CanonReturn SaveConfig(const char *file);
              params_->cmd = CmdSaveConfig;
            }
          } //if (strcmp (nameiter->c_str(), "type") == 0)
        } //for (; nameiter != attr.name.end(); ++nameiter, ++valiter)
      } //if (strcmp (tagName.c_str(), "CRPICommand") == 0)
      else if (strcmp(tagName.c_str(), "String") == 0)
      {
        //<String Value="schunk_hand"/>
        for (; nameiter != attr.name.end(); ++nameiter, ++valiter)
        {
          if (strcmp(nameiter->c_str(), "Value") == 0)
          {
            params_->str = valiter->c_str();
          }
        }
      } // else if (strcmp(tagName.c_str(), "String") == 0)
      else if (strcmp(tagName.c_str(), "Real") == 0)
      {
        //<Real Value="0.25" />
        for (; nameiter != attr.name.end(); ++nameiter, ++valiter)
        {
          if (strcmp(nameiter->c_str(), "Value") == 0)
          {
            params_->real = atof(valiter->c_str());
          }
        }
      } // else if (strcmp(tagName.c_str(), "Real") == 0)
      else if (strcmp(tagName.c_str(), "Int") == 0)
      {
        //<Int Value="3" />
        for (; nameiter != attr.name.end(); ++nameiter, ++valiter)
        {
          if (strcmp(nameiter->c_str(), "Value") == 0)
          {
            params_->integer = atoi(valiter->c_str());
          }
        }
      } //else if (strcmp(tagName.c_str(), "Int") == 0)
      else if (strcmp(tagName.c_str(), "Boolean") == 0)
      {
        //<Boolean Value="true" />
        for (; nameiter != attr.name.end(); ++nameiter, ++valiter)
        {
          if (strcmp(nameiter->c_str(), "Value") == 0)
          {
            params_->boolean = (strcmp(valiter->c_str(), "true") == 0);
          }
        }
      } //else if (strcmp(tagName.c_str(), "Boolean") == 0)
      else if (strcmp(tagName.c_str(), "Pose") == 0)
      {
        //<Pose X="2.5" Y="1.0" Z="1.0" XRot="90.0" YRot="0.0" ZRot="0.0" />
        for (; nameiter != attr.name.end(); ++nameiter, ++valiter)
        {
          if (strcmp(nameiter->c_str(), "X") == 0)
          {
            params_->pose->x = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "Y") == 0)
          {
            params_->pose->y = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "Z") == 0)
          {
            params_->pose->z = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "XRot") == 0)
          {
            params_->pose->xrot = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "YRot") == 0)
          {
            params_->pose->yrot = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "ZRot") == 0)
          {
            params_->pose->zrot = atof(valiter->c_str());
          }
        }
      }//<Pose X="2.5" Y="1.0" Z="1.0" XRot="90.0" YRot="0.0" ZRot="0.0" />
      else if (strcmp(tagName.c_str(), "Axes") == 0)
      {
        //<Axes J0="0.0" J1="0.0" J2="0.0" J3="0.0" J4="0.0" J5="0.0" J6="0.0" />
        for (; nameiter != attr.name.end(); ++nameiter, ++valiter)
        {
          if (strcmp(nameiter->c_str(), "J0") == 0)
          {
            params_->axes->axis.at(0) = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "J1") == 0)
          {
            params_->axes->axis.at(1) = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "J2") == 0)
          {
            params_->axes->axis.at(2) = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "J3") == 0)
          {
            params_->axes->axis.at(3) = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "J4") == 0)
          {
            params_->axes->axis.at(4) = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "J5") == 0)
          {
            params_->axes->axis.at(5) = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "J6") == 0)
          {
            params_->axes->axis.at(6) = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "J7") == 0)
          {
            params_->axes->axis.at(7) = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "J8") == 0)
          {
            params_->axes->axis.at(8) = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "J9") == 0)
          {
            params_->axes->axis.at(9) = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "J10") == 0)
          {
            params_->axes->axis.at(10) = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "J11") == 0)
          {
            params_->axes->axis.at(11) = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "J12") == 0)
          {
            params_->axes->axis.at(12) = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "J13") == 0)
          {
            params_->axes->axis.at(13) = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "J14") == 0)
          {
            params_->axes->axis.at(14) = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "J15") == 0)
          {
            params_->axes->axis.at(15) = atof(valiter->c_str());
          }
        }
      } // else if (strcmp(tagName.c_str(), "Axes") == 0)
      else if (strcmp(tagName.c_str(), "Matrix4x4") == 0)
      {
        //<Matrix4x4 V00="0.0" V01="0.0" V02="0.0" V03="0.0" V10="0.0" V11="0.0" V12="0.0" V13="0.0" V20="0.0" V21="0.0" V22="0.0" V23="0.0" V30="0.0" V31="0.0" V32="0.0" V33="0.0" />
        for (; nameiter != attr.name.end(); ++nameiter, ++valiter)
        {
          if (strcmp(nameiter->c_str(), "V00") == 0)
          {
            params_->matrx->at(0, 0) = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "V01") == 0)
          {
            params_->matrx->at(0, 1) = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "V02") == 0)
          {
            params_->matrx->at(0, 2) = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "V03") == 0)
          {
            params_->matrx->at(0, 3) = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "V10") == 0)
          {
            params_->matrx->at(1, 0) = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "V11") == 0)
          {
            params_->matrx->at(1, 1) = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "V12") == 0)
          {
            params_->matrx->at(1, 2) = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "V13") == 0)
          {
            params_->matrx->at(1, 3) = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "V20") == 0)
          {
            params_->matrx->at(2, 0) = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "V21") == 0)
          {
            params_->matrx->at(2, 1) = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "V22") == 0)
          {
            params_->matrx->at(2, 2) = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "V23") == 0)
          {
            params_->matrx->at(2, 3) = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "V30") == 0)
          {
            params_->matrx->at(3, 0) = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "V31") == 0)
          {
            params_->matrx->at(3, 1) = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "V32") == 0)
          {
            params_->matrx->at(3, 2) = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "V33") == 0)
          {
            params_->matrx->at(3, 3) = atof(valiter->c_str());
          }
        }
      } // else if (strcmp(tagName.c_str(), "Matrix4x4") == 0)
      else if (strcmp(tagName.c_str(), "RobotIO") == 0)
      {
        //! TODO
      } //else if (strcmp(tagName.c_str(), "RobotIO") == 0)
      else if (strcmp(tagName.c_str(), "Vector") == 0)
      {
        /*
        <Vector type="Real/Int/String/Bool">
          <Element value="<value>"\>
          ...
        </Vector>
        */
        //! TODO
      }
      else if (strcmp(tagName.c_str(), "Element") == 0)
      {
        //! TODO
      }
    } // try
    catch (...)
    {
      flag = false;
    }

    return flag;
  }


  LIBRARY_API bool CrpiXml::interTagElement (const string& tagName, 
                                             const std::vector<std::string>& vals)
  {
    bool flag = true;
    std::vector<std::string>::const_iterator valiter = vals.begin();

    try
    {
      /*
      //! Robot pose:
      if (strcmp (tagName.c_str(), "X") == 0)
      {
        for (; valiter != vals.end(); ++valiter)
        {
          params_->pose->x = atof (valiter->c_str ());
        }
      }
      if (strcmp (tagName.c_str(), "Y") == 0)
      {
        for (; valiter != vals.end(); ++valiter)
        {
          params_->pose->y = atof (valiter->c_str ());
        }
      }
      if (strcmp (tagName.c_str(), "Z") == 0)
      {
        for (; valiter != vals.end(); ++valiter)
        {
          params_->pose->z = atof (valiter->c_str ());
        }
      }
      */
      //! Robot orientation
      if (strcmp (tagName.c_str(), "I") == 0)
      {
        for (; valiter != vals.end(); ++valiter)
        {
          /*
          if (xaxisactive)
          {
            params_->xaxis.i = atof (valiter->c_str ());
          }
          else if (zaxisactive)
          {
            params_->zaxis.i = atof (valiter->c_str ());
          }
          */
        }
      }
      if (strcmp (tagName.c_str(), "J") == 0)
      {
        for (; valiter != vals.end(); ++valiter)
        {
          /*
          if (xaxisactive)
          {
            params_->xaxis.j = atof (valiter->c_str ());
          }
          else if (zaxisactive)
          {
            params_->zaxis.j = atof (valiter->c_str ());
          }
          */
        }
      }
      if (strcmp (tagName.c_str(), "K") == 0)
      {
        for (; valiter != vals.end(); ++valiter)
        {
          /*
          if (xaxisactive)
          {
            params_->xaxis.k = atof (valiter->c_str ());
          }
          else if (zaxisactive)
          {
            params_->zaxis.k = atof (valiter->c_str ());
          }
          */
        }
      }




      if (strcmp (tagName.c_str(), "Name") == 0)
      {
        for (; valiter != vals.end(); ++valiter)
        {
          params_->str = valiter->c_str();
        }
      }
    } // try
    catch (...)
    {
      flag = false;
    }

    return flag;
  }


  LIBRARY_API bool CrpiXml::endElement(const string& tagName)
  {
    /*
    if (strcmp (tagName.c_str(), "XAxis") == 0)
    {
      xaxisactive = false;
    } //if (strcmp (tagName.c_str(), "XAxis") == 0)
    if (strcmp (tagName.c_str(), "ZAxis") == 0)
    {
      zaxisactive = false;
    } //if (strcmp (tagName.c_str(), "ZAxis") == 0)
    */
    /*
    else
    {
      //! Unknown/unsupported XML tag
      return false;
    }
    */
    return true;
  }


  LIBRARY_API bool CrpiXml::encode (char *line)
  {
    std::string str;
    stringstream ss;
    ss.str(string());
    int i;

    switch (params_->status)
    {
    case CANON_SUCCESS:
      str = "Done";
      break;
    case CANON_REJECT:
      str = "Error";
      break;
    case CANON_FAILURE:
      str = "Error";
      break;
    case CANON_RUNNING:
      str = "Working";
      break;
    default:
      return CANON_FAILURE;
    }

    if (params_ != NULL)
    {
      ss << "<CRPIStatus>";
      ss << "<StatusID>" << params_->counter << "</StatusID><CommandState>" << str.c_str() << "</CommandState>";
      ss << "<Tool><Name>" << params_->toolName.c_str() << "</Name><Value>" << params_->toolVal << "</Value></Tool>";
      ss << "<Pose>" << "<X>" << params_->pose->x << "</X><Y>" << params_->pose->y << "</Y><Z>" << params_->pose->z
         << "</Z><XRot>" << params_->pose->xrot << "</XRot><YRot>" << params_->pose->yrot << "</YRot><ZRot>"
         << params_->pose->zrot << "</ZRot></Pose>";
      ss << "<Joints>";
      for (i = 0; i < params_->axes->axes; ++i)
      {
        ss << "<J" << i << ">" << params_->axes->axis.at(i) << "</J" << i << ">";
      }
      ss << "</Joints>";

      if (params_->cmd == CmdGetRobotForces)
      {

      }
      else if (params_->cmd == CmdGetRobotIO)
      {

      }
      else if (params_->cmd == CmdGetRobotSpeed)
      {

      }
      else if (params_->cmd == CmdGetRobotTorques)
      {
      }
      ss << "</CRPIStatus>";

/*
      //! Include gripper status if a tool has been defined using the couple command
      sprintf (line, "<CRPIStatus><StatusID>%d</StatusID><CommandState>%s</CommandState></CommandStatus><Tool><Name>%s</Name><Value>%f</Value></Gripper><Pose><X>%f</X><Y>%f</Y><Z>%f</Z><XRot>%f</XRot><YRot>%f</YRot><ZRot>%f</ZRot></Pose><Joints></Joints></CRPIStatus>\0",
                params_->counter,      
                str.c_str(),
                params_->toolName.c_str(),
                params_->toolVal,
                params_->pose->x,
                params_->pose->y,
                params_->pose->z,
                params_->pose->xrot,
                params_->pose->yrot,
                params_->pose->zrot,
                params_->axes->axis.at(0),
                params_->axes->axis.at(1),
                params_->axes->axis.at(2),
                params_->axes->axis.at(3),
                params_->axes->axis.at(4),
                params_->axes->axis.at(5),
                params_->axes->axis.at(6));
*/
    } //if (params_ != NULL)

    sprintf(line, "%s", ss.str().c_str());
    return true;
  }

} // XML