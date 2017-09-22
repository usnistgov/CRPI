///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Robot Interface
//  Workfile:        crcl_xml.cpp
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
  LIBRARY_API CrclXml::CrclXml (CrpiXmlParams *params) :
    params_(params)
  {
    xaxisactive = zaxisactive = false;
  }


  LIBRARY_API CrclXml::~CrclXml ()
  {
    params_ = NULL;
  }


  LIBRARY_API bool CrclXml::characters(const vector<string>& ch)
  {
    vector<string>::const_iterator chIter = ch.begin();

    for (; chIter != ch.end(); ++chIter)
    {
    }

    return true;
  }


  LIBRARY_API bool CrclXml::parse (const string &line)
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
    Example command for robot motion:

    <?xml version="1.0" encoding="UTF-8"?>
    <CRCLCommandInstance
     xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
     xsi:noNamespaceSchemaLocation="../xmlSchemas/CRCLCommandInstance.xsd">
      <CRCLCommand xsi:type="MoveToType">
        <CommandID>2</CommandID>
        <MoveStraight>false</MoveStraight>
        <Pose>
          <Point>
            <X>2.5</X> <Y>1</Y> <Z>1</Z>
          </Point>
          <XAxis>
            <I>1</I> <J>0</J> <K>0</K>
          </XAxis>
          <ZAxis>
            <I>0</I> <J>0</J> <K>-1</K>
          </ZAxis>
        </Pose>
        <NumPositions>1</NumPositions>
      </CRCLCommand>
    </CRCLCommandInstance>

    Example command for gripper:
    <?xml version="1.0" encoding="UTF-8"?>
    <CRCLCommandInstance xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
      <CRCLCommand xsi:type="SetEndEffectorType">
        <CommandID>123</CommandID>
        <NumPositions>0.5</NumPositions>
      </CRCLCommand>
    </CRCLCommandInstance>
  */

  LIBRARY_API bool CrclXml::startElement (const string& tagName, 
                                          const xmlAttributes& attr)
  {
    bool flag = true;
    std::vector<std::string>::const_iterator nameiter = attr.name.begin();
    std::vector<std::string>::const_iterator valiter = attr.val.begin();

    try
    {
      if (strcmp (tagName.c_str(), "CRCLCommand") == 0)
      {
        for (; nameiter != attr.name.end(); ++nameiter, ++valiter)
        {
          if (strcmp (nameiter->c_str(), "xsi:type") == 0)
          {
            if (strcmp (valiter->c_str(), "ActuateJointsType") == 0)
            {
              params_->cmd = CmdMoveToAxisTarget;
            }
            else if (strcmp (valiter->c_str(), "CloseToolChangerType") == 0)
            {
              params_->cmd = CmdCouple;
            }
            else if (strcmp (valiter->c_str(), "ConfigureJointReportsType") == 0)
            {
              //! JAM: No CRPI equivalent
            }
            else if (strcmp (valiter->c_str(), "DwellType") == 0)
            {
              //! No CRPI equivalent
              params_->cmd = CmdDwell;
            }
            else if (strcmp (valiter->c_str(), "EndCanonType") == 0)
            {
              //! No CRPI equivalent
              params_->cmd = CmdEndCanon;
            }
            else if (strcmp (valiter->c_str(), "GetStatusType") == 0)
            {
              params_->cmd = CmdGetRobotPose;
            }
            else if (strcmp (valiter->c_str(), "InitCanonType") == 0)
            {
              params_->cmd = CmdInitCanon;
            }
            else if (strcmp (valiter->c_str(), "MessageType") == 0)
            {
              params_->cmd = CmdMessage;
            }
            else if (strcmp (valiter->c_str(), "MoveScrewType") == 0)
            {
              //! JAM:  No CRPI equivalent
            }
            else if (strcmp (valiter->c_str(), "MoveThroughToType") == 0)
            {
              params_->cmd = CmdMoveThroughTo;
            }
            else if (strcmp (valiter->c_str(), "MoveToType") == 0)
            {
              params_->cmd = CmdMoveTo;
            }
            else if (strcmp (valiter->c_str(), "RunProgramType") == 0)
            {
              //! JAM:  No CRPI equivalent
              params_->cmd = CmdRunProgram;
            }
            else if (strcmp (valiter->c_str(), "SetAbsoluteAccelerationType") == 0)
            {
              params_->cmd = CmdSetAbsoluteAcceleration;
            }
            else if (strcmp (valiter->c_str(), "SetAbsoluteSpeedType") == 0)
            {
              params_->cmd = CmdSetAbsoluteSpeed;
            }
            else if (strcmp (valiter->c_str(), "SetAngleUnitsType") == 0)
            {
              params_->cmd = CmdSetAngleUnits;
            }
            else if (strcmp (valiter->c_str(), "SetEndEffectorParametersType") == 0)
            {
              params_->cmd = CmdSetParameter;
            }
            else if (strcmp (valiter->c_str(), "SetEndEffectorType") == 0)
            {
              params_->cmd = CmdSetTool;
            }
            else if (strcmp (valiter->c_str(), "SetEndPoseToleranceType") == 0)
            {
              params_->cmd = CmdSetEndPoseTolerance;
            }
            else if (strcmp (valiter->c_str(), "SetForceUnitsType") == 0)
            {
              //! JAM: No CRPI equivalent
            }
            else if (strcmp (valiter->c_str(), "SetIntermediatePoseToleranceType") == 0)
            {
              params_->cmd = CmdSetIntermediatePoseTolerance;
            }
            else if (strcmp (valiter->c_str(), "SetJointControlModesType") == 0)
            {
              //! JAM:  No CRPI equivalent
            }
            else if (strcmp (valiter->c_str(), "SetLengthUnitsType") == 0)
            {
              params_->cmd = CmdSetLengthUnits;
            }
            else if (strcmp (valiter->c_str(), "SetRelativeAccelerationType") == 0)
            {
              params_->cmd = CmdSetRelativeAcceleration;
            }
            else if (strcmp (valiter->c_str(), "SetTransSpeedRelativeType") == 0)
            {
              params_->cmd = CmdSetRelativeSpeed;
            }
            else if (strcmp (valiter->c_str(), "SetRobotParametersType") == 0)
            {
              params_->cmd = CmdSetParameter;
            }
            else if (strcmp (valiter->c_str(), "SetTorqueUnitsType") == 0)
            {
              //! JAM: No CRPI equivalent
            }
            else if (strcmp (valiter->c_str(), "StopMotionType") == 0)
            {
              params_->cmd = CmdStopMotion;
            }
          } //if (strcmp (nameiter->c_str(), "xsi:type") == 0)
        } //for (; nameiter != attr.name.end(); ++nameiter, ++valiter)
      } //if (strcmp (tagName.c_str(), "CRCLCommand") == 0)

      /*
        All other variables set using the interTagElement function
      */

      if (strcmp (tagName.c_str(), "XAxis") == 0)
      {
        xaxisactive = true;
      } //if (strcmp (tagName.c_str(), "XAxis") == 0)
      if (strcmp (tagName.c_str(), "ZAxis") == 0)
      {
        zaxisactive = true;
      } //if (strcmp (tagName.c_str(), "ZAxis") == 0)

    } // try
    catch (...)
    {
      flag = false;
    }

    return flag;
  }


  LIBRARY_API bool CrclXml::interTagElement (const string& tagName, 
                                             const std::vector<std::string>& vals)
  {
    bool flag = true;
    std::vector<std::string>::const_iterator valiter = vals.begin();

    try
    {
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

      //! Robot orientation
      if (strcmp (tagName.c_str(), "I") == 0)
      {
        for (; valiter != vals.end(); ++valiter)
        {
          if (xaxisactive)
          {
            params_->xaxis.i = atof (valiter->c_str ());
          }
          else if (zaxisactive)
          {
            params_->zaxis.i = atof (valiter->c_str ());
          }
        }
      }
      if (strcmp (tagName.c_str(), "J") == 0)
      {
        for (; valiter != vals.end(); ++valiter)
        {
          if (xaxisactive)
          {
            params_->xaxis.j = atof (valiter->c_str ());
          }
          else if (zaxisactive)
          {
            params_->zaxis.j = atof (valiter->c_str ());
          }
        }
      }
      if (strcmp (tagName.c_str(), "K") == 0)
      {
        for (; valiter != vals.end(); ++valiter)
        {
          if (xaxisactive)
          {
            params_->xaxis.k = atof (valiter->c_str ());
          }
          else if (zaxisactive)
          {
            params_->zaxis.k = atof (valiter->c_str ());
          }
        }
      }

      //! NumPositions
      if (strcmp (tagName.c_str(), "NumPositions") == 0)
      {
        for (; valiter != vals.end(); ++valiter)
        {
          params_->numPositions = atof (valiter->c_str ());
        }
      }

      //! CommandID
      if (strcmp (tagName.c_str(), "CommandID") == 0)
      {
        for (; valiter != vals.end(); ++valiter)
        {
          params_->commandID = atoi (valiter->c_str ());
        }
      }

      //! Setting
      if (strcmp (tagName.c_str(), "Setting") == 0)
      {
        for (; valiter != vals.end(); ++valiter)
        {
          params_->setting = atof (valiter->c_str ());
        }
      }

      //! MoveStraight
      if (strcmp (tagName.c_str(), "MoveStraight") == 0)
      {
        for (; valiter != vals.end(); ++valiter)
        {
          if (strcmp (valiter->c_str(), "true"))
          {
            params_->moveStraight = true;
          }
          else
          {
            params_->moveStraight = false;
          }
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


  LIBRARY_API bool CrclXml::endElement(const string& tagName)
  {
    if (strcmp (tagName.c_str(), "XAxis") == 0)
    {
      xaxisactive = false;
    } //if (strcmp (tagName.c_str(), "XAxis") == 0)
    if (strcmp (tagName.c_str(), "ZAxis") == 0)
    {
      zaxisactive = false;
    } //if (strcmp (tagName.c_str(), "ZAxis") == 0)

    /*
    if (tagName == "RobData")
    {
    }
    else
    {
      //! Unknown/unsupported XML tag
      return false;
    }
    */
    return true;
  }

  /*
  <?xml version="1.0" encoding="UTF-8"?>
  <CRCLStatus xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="../xmlSchemas/CRCLStatus.xsd">
    <CommandStatus>
      <CommandID>1</CommandID>
      <StatusID>1</StatusID>
      <CommandState>Working</CommandState>
    </CommandStatus>
    <Pose>
      <Point>
        <X>1.5</X> <Y>1</Y> <Z>1</Z>
      </Point>
      <XAxis>
        <I>1</I> <J>0</J> <K>0</K>
      </XAxis>
      <ZAxis>
        <I>0</I> <J>0</J> <K>-1</K>
      </ZAxis>
    </Pose>
  </CRCLStatus>
  */
  LIBRARY_API bool CrclXml::encode (char *line)
  {
    std::string str;
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
      if (strcmp (params_->toolName.c_str(), "Nothing") == 0)
      {
        sprintf (line, "<?xml version=\"1.0\" encoding=\"UTF-8\"?><CRCLStatus xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:noNamespaceSchemaLocation=\"../xmlSchemas/CRCLStatus.xsd\"><CommandStatus><CommandID>%d</CommandID><StatusID>%d</StatusID><CommandState>%s</CommandState></CommandStatus><Pose><Point><X>%f</X><Y>%f</Y><Z>%f</Z></Point><XAxis><I>%f</I><J>%f</J><K>%f</K></XAxis><ZAxis><I>%f</I><J>%f</J><K>%f</K></ZAxis></Pose></CRCLStatus>\0",
                 params_->commandID,
                 params_->counter,
                 str.c_str(),
                 params_->pose->x,
                 params_->pose->y,
                 params_->pose->z,
                 params_->xaxis.i,
                 params_->xaxis.j,
                 params_->xaxis.k,
                 params_->zaxis.i,
                 params_->zaxis.j,
                 params_->zaxis.k);
      }
      else
      {
        //! Include gripper status if a tool has been defined using the couple command
        sprintf (line, "<?xml version=\"1.0\" encoding=\"UTF-8\"?><CRCLStatus xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:noNamespaceSchemaLocation=\"../xmlSchemas/CRCLStatus.xsd\"><CommandStatus><CommandID>%d</CommandID><StatusID>%d</StatusID><CommandState>%s</CommandState></CommandStatus><GripperStatus><GripperName>%s</GripperName><Separation>%f</Separation></GripperStatus><Pose><Point><X>%f</X><Y>%f</Y><Z>%f</Z></Point><XAxis><I>%f</I><J>%f</J><K>%f</K></XAxis><ZAxis><I>%f</I><J>%f</J><K>%f</K></ZAxis></Pose></CRCLStatus>\0",
                 params_->commandID,
                 params_->counter,      
                 str.c_str(),
                 params_->toolName.c_str(),
                 params_->toolVal,
                 params_->pose->x,
                 params_->pose->y,
                 params_->pose->z,
                 params_->xaxis.i,
                 params_->xaxis.j,
                 params_->xaxis.k,
                 params_->zaxis.i,
                 params_->zaxis.j,
                 params_->zaxis.k);
      }

    } //if (params_ != NULL)
    return true;
  }

} // XML