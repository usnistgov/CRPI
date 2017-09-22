///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       XML
//  Workfile:        crpi_robot_xml.cpp
//  Revision:        1.0 - 17 June, 2015
//
//  Author:          J. Marvel
//
//  Description
//  ===========
//  XML parser class definition file for CRPI robot configurations.
//
///////////////////////////////////////////////////////////////////////////////

#include "crpi_robot_xml.h"
#include <iostream>
#include <sstream>

using namespace std;

namespace Xml
{
  LIBRARY_API CrpiRobotXml::CrpiRobotXml (CrpiRobotParams *params) :
    params_(params)
  {
    toolTmp = new CrpiToolDef;
  }


  LIBRARY_API CrpiRobotXml::~CrpiRobotXml ()
  {
    params_ = NULL;
    delete toolTmp;
  }


  LIBRARY_API bool CrpiRobotXml::characters(const vector<string>& ch)
  {
    vector<string>::const_iterator chIter = ch.begin();

    for (; chIter != ch.end(); ++chIter)
    {
    }

    return true;
  }


  LIBRARY_API bool CrpiRobotXml::parse (const string &line)
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
  <Robot>
    <TCP_IP Address="127.0.0.1" Port="6007" Client="false"/>
    <Serial Port="COM7" Rate="57600" Parity="Even" SBits="1" Handshake="None"/>
    <ComType Val="Serial"/>
    <Observer Address="169.254.152.3" Port="1025" Client="true"/>
    <Mounting X="0.0" Y="0.0" Z="0.0" XR="0.0" YR="0.0" ZR="0.0"/>
    <ToWorld X="2335.14" Y="471.0" Z="661.0" XR="0.0" YR="0.0" ZR="90.0" M00="0.0" M01="0.0" M02="0.0" M03="0.0" M10="0.0" M11="0.0" M12="0.0" M13="0.0" M20="0.0" M21="0.0" M22="0.0" M23="0.0" M30="0.0" M31="0.0" M32="0.0" M33="0.0"/>
    <CoordSystem Name="Table1" X="2335.14" Y="471.0" Z="661.0" XR="0.0" YR="0.0" ZR="90.0" M00="0.0" M01="0.0" M02="0.0" M03="0.0" M10="0.0" M11="0.0" M12="0.0" M13="0.0" M20="0.0" M21="0.0" M22="0.0" M23="0.0" M30="0.0" M31="0.0" M32="0.0" M33="0.0"/>
    <CoordSystem Name="Table2" X="2335.14" Y="471.0" Z="661.0" XR="0.0" YR="0.0" ZR="90.0" M00="0.0" M01="0.0" M02="0.0" M03="0.0" M10="0.0" M11="0.0" M12="0.0" M13="0.0" M20="0.0" M21="0.0" M22="0.0" M23="0.0" M30="0.0" M31="0.0" M32="0.0" M33="0.0"/>
    <Tool ID="7" Name="gripper_gear" X="0.0" Y="0.0" Z="0.0" XR="0.0" YR="0.0" ZR="0.0" MX="0.0" MY="0.0" MZ="0.0/>
    <Tool ID="7" Name="gripper_top_cover" X="0.0" Y="0.0" Z="0.0" XR="0.0" YR="0.0" ZR="0.0" MX="0.0" MY="0.0" MZ="0.0/>
    <Tool ID="7" Name="gripper_bottom_cover" X="0.0" Y="0.0" Z="0.0" XR="0.0" YR="0.0" ZR="0.0" MX="0.0" MY="0.0" MZ="0.0/>
    <Tool ID="4" Name="gripper_parallel" X="0.0" Y="0.0" Z="0.0" XR="0.0" YR="0.0" ZR="0.0" MX="0.0" MY="0.0" MZ="0.0/>
    <Tool ID="3" Name="schunk_hand" X="0.0" Y="0.0" Z="0.0" XR="0.0" YR="0.0" ZR="0.0" MX="0.0" MY="0.0" MZ="0.0/>
  </Robot>
  */

  LIBRARY_API bool CrpiRobotXml::startElement (const string& tagName, 
                                               const xmlAttributes& attr)
  {
    std::vector<std::string>::const_iterator nameiter = attr.name.begin();
    std::vector<std::string>::const_iterator valiter = attr.val.begin();
    bool flag = true;

    try
    {
      if (strcmp (tagName.c_str(), "Robot") == 0)
      {
      } // if (strcmp (tagName.c_str(), "Robot") == 0)
      else if (strcmp (tagName.c_str(), "TCP_IP") == 0)
      {
        //! <TCP_IP Address="127.0.0.1" Port="6007" Client="false"/>
        for (; nameiter != attr.name.end(); ++nameiter, ++valiter)
        {
          if (strcmp (nameiter->c_str(), "Address") == 0)
          {
            sprintf(params_->tcp_ip_addr, valiter->c_str());
          }
          else if (strcmp (nameiter->c_str(), "Port") == 0)
          {
            params_->tcp_ip_port = atoi (valiter->c_str ());
          }
          else if (strcmp (nameiter->c_str(), "Client") == 0)
          {
            params_->tcp_ip_client = (strcmp (valiter->c_str(), "true") == 0);
          }
          else
          {
            //! Unknown tag
          }
        } //for (; nameiter != attr.name.end(); ++nameiter, ++valiter)
      } //... else if (strcmp (tagName.c_str(), "TCP_IP") == 0)
      else if (strcmp(tagName.c_str(), "Observer") == 0)
      {
        //! <TCP_IP Address="127.0.0.1" Port="6007" Client="false"/>
        for (; nameiter != attr.name.end(); ++nameiter, ++valiter)
        {
          if (strcmp(nameiter->c_str(), "Address") == 0)
          {
            sprintf(params_->obs_tcp_ip_addr, valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "Port") == 0)
          {
            params_->obs_tcp_ip_port = atoi(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "Client") == 0)
          {
            params_->obs_tcp_ip_client = (strcmp(valiter->c_str(), "true") == 0);
          }
          else
          {
            //! Unknown tag
          }
        } //for (; nameiter != attr.name.end(); ++nameiter, ++valiter)
      } //... else if (strcmp (tagName.c_str(), "Observer") == 0)
      else if (strcmp (tagName.c_str(), "Serial") == 0)
      {
        //! <Serial Port="COM7" Rate="57600" Parity="Even" SBits="1" Handshake="None"/>
        for (nameiter = attr.name.begin(), valiter = attr.val.begin(); nameiter != attr.name.end(); ++nameiter, ++valiter)
        {
          if (strcmp (nameiter->c_str(), "Port") == 0)
          {
            sprintf(params_->serial_port, valiter->c_str());
          }
          else if (strcmp (nameiter->c_str(), "Rate") == 0)
          {
            params_->serial_rate = atoi (valiter->c_str ());
          }
          else if (strcmp (nameiter->c_str(), "Parity") == 0)
          {
            params_->serial_parity_even = (strcmp (valiter->c_str(), "Even") == 0);
          }
          else if (strcmp (nameiter->c_str(), "SBits") == 0)
          {
            params_->serial_sbits = atoi (valiter->c_str());
          }
          else if (strcmp (nameiter->c_str(), "Handshake") == 0)
          {
            sprintf (params_->serial_handshake, valiter->c_str());
          }
          else
          {
            //! Unknown tag
          }
        } //for (; nameiter != attr.name.end(); ++nameiter, ++valiter)
      } //else if (strcmp (tagName.c_str(), "Serial") == 0)
      else if (strcmp (tagName.c_str(), "ComType") == 0)
      {
        //! <ComType Val="Serial"/>
        for (nameiter = attr.name.begin(), valiter = attr.val.begin(); nameiter != attr.name.end(); ++nameiter, ++valiter)
        {
          if (strcmp (nameiter->c_str(), "Val") == 0)
          {
            params_->use_serial = (strcmp (valiter->c_str(), "Serial") == 0);
          }
          else
          {
            //! Unknown tag
          }
        } //for (; nameiter != attr.name.end(); ++nameiter, ++valiter)
      } //else if (strcmp (tagName.c_str(), "ComType") == 0)
      else if (strcmp (tagName.c_str(), "Mounting") == 0)
      {
        //! <Mounting X="0.0" Y="0.0" Z="0.0" XR="0.0" YR="0.0" ZR="0.0"/>
        for (nameiter = attr.name.begin(), valiter = attr.val.begin(); nameiter != attr.name.end(); ++nameiter, ++valiter)
        {
          if (strcmp (nameiter->c_str(), "X") == 0)
          {
            params_->mounting->x = atof(valiter->c_str());
          }
          else if (strcmp (nameiter->c_str(), "Y") == 0)
          {
            params_->mounting->y = atof(valiter->c_str());
          }
          else if (strcmp (nameiter->c_str(), "Z") == 0)
          {
            params_->mounting->z = atof(valiter->c_str());
          }
          else if (strcmp (nameiter->c_str(), "XR") == 0)
          {
            params_->mounting->xrot = atof(valiter->c_str());
          }
          else if (strcmp (nameiter->c_str(), "YR") == 0)
          {
            params_->mounting->yrot = atof(valiter->c_str());
          }
          else if (strcmp (nameiter->c_str(), "ZR") == 0)
          {
            params_->mounting->zrot = atof(valiter->c_str());
          }
          else
          {
            //! Unknown tag
          }
        } //for (; nameiter != attr.name.end(); ++nameiter, ++valiter)
      } //else if (strcmp (tagName.c_str(), "Mounting") == 0)
      else if (strcmp (tagName.c_str(), "ToWorld") == 0)
      {
        // <ToWorld X = "2335.14" Y = "471.0" Z = "661.0" XR = "0.0" YR = "0.0" ZR = "90.0" M00 = "0.0" M01 = "0.0" M02 = "0.0" M03 = "0.0" M10 = "0.0" M11 = "0.0" M12 = "0.0" M13 = "0.0" M20 = "0.0" M21 = "0.0" M22 = "0.0" M23 = "0.0" M30 = "0.0" M31 = "0.0" M32 = "0.0" M33 = "0.0" / >
        for (nameiter = attr.name.begin(), valiter = attr.val.begin(); nameiter != attr.name.end(); ++nameiter, ++valiter)
        {
          if (strcmp (nameiter->c_str(), "X") == 0)
          {
            params_->toWorld->x = atof(valiter->c_str());
          }
          else if (strcmp (nameiter->c_str(), "Y") == 0)
          {
            params_->toWorld->y = atof(valiter->c_str());
          }
          else if (strcmp (nameiter->c_str(), "Z") == 0)
          {
            params_->toWorld->z = atof(valiter->c_str());
          }
          else if (strcmp (nameiter->c_str(), "XR") == 0)
          {
            params_->toWorld->xrot = atof(valiter->c_str());
          }
          else if (strcmp (nameiter->c_str(), "YR") == 0)
          {
            params_->toWorld->yrot = atof(valiter->c_str());
          }
          else if (strcmp (nameiter->c_str(), "ZR") == 0)
          {
            params_->toWorld->zrot = atof(valiter->c_str());
          }
          else if (strcmp (nameiter->c_str(), "M00") == 0)
          {
            params_->toWorldMatrix->at(0, 0) = atof(valiter->c_str());
            params_->usedMatrix = true;
          }
          else if (strcmp (nameiter->c_str(), "M01") == 0)
          {
            params_->toWorldMatrix->at(0, 1) = atof(valiter->c_str());
            params_->usedMatrix = true;
          }
          else if (strcmp (nameiter->c_str(), "M02") == 0)
          {
            params_->toWorldMatrix->at(0, 2) = atof(valiter->c_str());
            params_->usedMatrix = true;
          }
          else if (strcmp (nameiter->c_str(), "M03") == 0)
          {
            params_->toWorldMatrix->at(0, 3) = atof(valiter->c_str());
            params_->usedMatrix = true;
          }
          else if (strcmp (nameiter->c_str(), "M10") == 0)
          {
            params_->toWorldMatrix->at(1, 0) = atof(valiter->c_str());
            params_->usedMatrix = true;
          }
          else if (strcmp (nameiter->c_str(), "M11") == 0)
          {
            params_->toWorldMatrix->at(1, 1) = atof(valiter->c_str());
            params_->usedMatrix = true;
          }
          else if (strcmp (nameiter->c_str(), "M12") == 0)
          {
            params_->toWorldMatrix->at(1, 2) = atof(valiter->c_str());
            params_->usedMatrix = true;
          }
          else if (strcmp (nameiter->c_str(), "M13") == 0)
          {
            params_->toWorldMatrix->at(1, 3) = atof(valiter->c_str());
            params_->usedMatrix = true;
          }
          else if (strcmp (nameiter->c_str(), "M20") == 0)
          {
            params_->toWorldMatrix->at(2, 0) = atof(valiter->c_str());
            params_->usedMatrix = true;
          }
          else if (strcmp (nameiter->c_str(), "M21") == 0)
          {
            params_->toWorldMatrix->at(2, 1) = atof(valiter->c_str());
            params_->usedMatrix = true;
          }
          else if (strcmp (nameiter->c_str(), "M22") == 0)
          {
            params_->toWorldMatrix->at(2, 2) = atof(valiter->c_str());
            params_->usedMatrix = true;
          }
          else if (strcmp (nameiter->c_str(), "M23") == 0)
          {
            params_->toWorldMatrix->at(2, 3) = atof(valiter->c_str());
            params_->usedMatrix = true;
          }
          else if (strcmp (nameiter->c_str(), "M30") == 0)
          {
            params_->toWorldMatrix->at(3, 0) = atof(valiter->c_str());
            params_->usedMatrix = true;
          }
          else if (strcmp (nameiter->c_str(), "M31") == 0)
          {
            params_->toWorldMatrix->at(3, 1) = atof(valiter->c_str());
            params_->usedMatrix = true;
          }
          else if (strcmp (nameiter->c_str(), "M32") == 0)
          {
            params_->toWorldMatrix->at(3, 2) = atof(valiter->c_str());
            params_->usedMatrix = true;
          }
          else if (strcmp (nameiter->c_str(), "M33") == 0)
          {
            params_->toWorldMatrix->at(3, 3) = atof(valiter->c_str());
            params_->usedMatrix = true;
          }
          else
          {
            //! Unknown tag
          }
        } //for (; nameiter != attr.name.end(); ++nameiter, ++valiter)
      } //else if (strcmp (tagName.c_str(), "ToWorld") == 0)
      else if (strcmp(tagName.c_str(), "CoordSystem") == 0)
      {
        // <CoordSystem Name = "Table1" X = "2335.14" Y = "471.0" Z = "661.0" XR = "0.0" YR = "0.0" ZR = "90.0" M00 = "0.0" M01 = "0.0" M02 = "0.0" M03 = "0.0" M10 = "0.0" M11 = "0.0" M12 = "0.0" M13 = "0.0" M20 = "0.0" M21 = "0.0" M22 = "0.0" M23 = "0.0" M30 = "0.0" M31 = "0.0" M32 = "0.0" M33 = "0.0" / >
        Math::matrix *mtemp;
        mtemp = new Math::matrix(4, 4);
        string stemp;
        Math::pose ptemp;
        bool usedmatrix = false;
        for (nameiter = attr.name.begin(), valiter = attr.val.begin(); nameiter != attr.name.end(); ++nameiter, ++valiter)
        {
          if (strcmp(nameiter->c_str(), "Name") == 0)
          {
            stemp = valiter->c_str();
          }
          if (strcmp(nameiter->c_str(), "X") == 0)
          {
            ptemp.x = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "Y") == 0)
          {
            ptemp.y = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "Z") == 0)
          {
            ptemp.z = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "XR") == 0)
          {
            ptemp.xr = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "YR") == 0)
          {
            ptemp.yr = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "ZR") == 0)
          {
            ptemp.zr = atof(valiter->c_str());
          }
          else if (strcmp(nameiter->c_str(), "M00") == 0)
          {
            mtemp->at(0, 0) = atof(valiter->c_str());
            usedmatrix = true;
          }
          else if (strcmp(nameiter->c_str(), "M01") == 0)
          {
            mtemp->at(0, 1) = atof(valiter->c_str());
            usedmatrix = true;
          }
          else if (strcmp(nameiter->c_str(), "M02") == 0)
          {
            mtemp->at(0, 2) = atof(valiter->c_str());
            usedmatrix = true;
          }
          else if (strcmp(nameiter->c_str(), "M03") == 0)
          {
            mtemp->at(0, 3) = atof(valiter->c_str());
            usedmatrix = true;
          }
          else if (strcmp(nameiter->c_str(), "M10") == 0)
          {
            mtemp->at(1, 0) = atof(valiter->c_str());
            usedmatrix = true;
          }
          else if (strcmp(nameiter->c_str(), "M11") == 0)
          {
            mtemp->at(1, 1) = atof(valiter->c_str());
            usedmatrix = true;
          }
          else if (strcmp(nameiter->c_str(), "M12") == 0)
          {
            mtemp->at(1, 2) = atof(valiter->c_str());
            usedmatrix = true;
          }
          else if (strcmp(nameiter->c_str(), "M13") == 0)
          {
            mtemp->at(1, 3) = atof(valiter->c_str());
            usedmatrix = true;
          }
          else if (strcmp(nameiter->c_str(), "M20") == 0)
          {
            mtemp->at(2, 0) = atof(valiter->c_str());
            usedmatrix = true;
          }
          else if (strcmp(nameiter->c_str(), "M21") == 0)
          {
            mtemp->at(2, 1) = atof(valiter->c_str());
            usedmatrix = true;
          }
          else if (strcmp(nameiter->c_str(), "M22") == 0)
          {
            mtemp->at(2, 2) = atof(valiter->c_str());
            usedmatrix = true;
          }
          else if (strcmp(nameiter->c_str(), "M23") == 0)
          {
            mtemp->at(2, 3) = atof(valiter->c_str());
            usedmatrix = true;
          }
          else if (strcmp(nameiter->c_str(), "M30") == 0)
          {
            mtemp->at(3, 0) = atof(valiter->c_str());
            usedmatrix = true;
          }
          else if (strcmp(nameiter->c_str(), "M31") == 0)
          {
            mtemp->at(3, 1) = atof(valiter->c_str());
            usedmatrix = true;
          }
          else if (strcmp(nameiter->c_str(), "M32") == 0)
          {
            mtemp->at(3, 2) = atof(valiter->c_str());
            usedmatrix = true;
          }
          else if (strcmp(nameiter->c_str(), "M33") == 0)
          {
            mtemp->at(3, 3) = atof(valiter->c_str());
            usedmatrix = true;
          }
          else
          {
            //! Unknown tag
          }
        } //for (; nameiter != attr.name.end(); ++nameiter, ++valiter)

        if (!usedmatrix)
        {
          //! Update to matrix representation
          mtemp->RPYMatrixConvert(ptemp, true);
        }
        robotPose rptemp;
        rptemp = ptemp;
        params_->toCoordSystMatrices.push_back(mtemp);
        params_->toCoordSystPoses.push_back(rptemp);
        params_->coordSystNames.push_back(stemp);
      } //else if (strcmp (tagName.c_str(), "CoordSystem") == 0)
      else if (strcmp (tagName.c_str(), "Tool") == 0)
      {
        //<Tool ID="7" Name="gripper_gear" X="0.0" Y="0.0" Z="0.0" XR="0.0" YR="0.0" ZR="0.0" MX="0.0" MY="0.0" MZ="0.0"/>
        for (nameiter = attr.name.begin(), valiter = attr.val.begin(); nameiter != attr.name.end(); ++nameiter, ++valiter)
        {
          if (strcmp (nameiter->c_str(), "ID") == 0)
          {
            toolTmp->toolID = atoi(valiter->c_str());
          }
          else if (strcmp (nameiter->c_str(), "Name") == 0)
          {
            toolTmp->toolName = valiter->c_str();
          }
          else if (strcmp (nameiter->c_str(), "X") == 0)
          {
            toolTmp->TCP.x = atof(valiter->c_str());
          }
          else if (strcmp (nameiter->c_str(), "Y") == 0)
          {
            toolTmp->TCP.y = atof(valiter->c_str());
          }
          else if (strcmp (nameiter->c_str(), "Z") == 0)
          {
            toolTmp->TCP.z = atof(valiter->c_str());
          }
          else if (strcmp (nameiter->c_str(), "Mass") == 0)
          {
            toolTmp->mass = atof(valiter->c_str());
          }
          else if (strcmp (nameiter->c_str(), "XR") == 0)
          {
            toolTmp->TCP.xrot = atof(valiter->c_str());
          }
          else if (strcmp (nameiter->c_str(), "YR") == 0)
          {
            toolTmp->TCP.yrot = atof(valiter->c_str());
          }
          else if (strcmp (nameiter->c_str(), "ZR") == 0)
          {
            toolTmp->TCP.zrot = atof(valiter->c_str());
          }
          else if (strcmp (nameiter->c_str(), "MX") == 0)
          {
            toolTmp->centerMass.x = atof(valiter->c_str());
          }
          else if (strcmp (nameiter->c_str(), "MY") == 0)
          {
            toolTmp->centerMass.y = atof(valiter->c_str());
          }
          else if (strcmp (nameiter->c_str(), "MZ") == 0)
          {
            toolTmp->centerMass.z = atof(valiter->c_str());
          }
          else
          {
            //! Unknown tag
          }
        } // for (; nameiter != attr.name.end(); ++nameiter, ++valiter)
      } //else if (strcmp (tagName.c_str(), "Tool") == 0)
      else
      {
        //! Unknown tag
      }
    } // try
    catch (...)
    {
      flag = false;
    }

    return flag;
  }


  LIBRARY_API bool CrpiRobotXml::interTagElement (const string& tagName, 
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
*/
    } // try
    catch (...)
    {
      flag = false;
    }

    return flag;
  }


  LIBRARY_API bool CrpiRobotXml::endElement(const string& tagName)
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
    if (tagName == "RobData")
    {
    }
    else
    {
      //! Unknown/unsupported XML tag
      return false;
    }
    */
    if (strcmp(tagName.c_str(), "Tool") == 0)
    {
      params_->tools.push_back(*toolTmp);
    }
    return true;
  }


  /*
  <ROBOT>
    <TCP_IP Address="169.254.152.38" Port="30003" Client="true">
    <ComType Val="TCP_IP"/>
    <Mounting X="0.0" Y="0.0" Z="0.0" XR="-135.0" YR="0.0" ZR="0.0"/>
    <ToWorld X="2335.14" Y="471.0" Z="661.0" XR="0.0" YR="0.0" ZR="90.0" M00="0.0" M01="0.0" M02="0.0" M03="0.0" M10="0.0" M11="0.0" M12="0.0" M13="0.0" M20="0.0" M21="0.0" M22="0.0" M23="0.0" M30="0.0" M31="0.0" M32="0.0" M33="0.0"/>
    <CoordSystem Name="Table1" X="2335.14" Y="471.0" Z="661.0" XR="0.0" YR="0.0" ZR="90.0" M00="0.0" M01="0.0" M02="0.0" M03="0.0" M10="0.0" M11="0.0" M12="0.0" M13="0.0" M20="0.0" M21="0.0" M22="0.0" M23="0.0" M30="0.0" M31="0.0" M32="0.0" M33="0.0"/>
    <CoordSystem Name="Table2" X="2335.14" Y="471.0" Z="661.0" XR="0.0" YR="0.0" ZR="90.0" M00="0.0" M01="0.0" M02="0.0" M03="0.0" M10="0.0" M11="0.0" M12="0.0" M13="0.0" M20="0.0" M21="0.0" M22="0.0" M23="0.0" M30="0.0" M31="0.0" M32="0.0" M33="0.0"/>
    <Tool ID="7" Name="gripper_gear" X="0.0" Y="0.0" Z="185.0" XR="0.0" YR="0.0" ZR="0.0" Mass="0.9" MX="0.0" MY="0.0" MZ="71.0"/>
    <Tool ID="7" Name="gripper_top_cover" X="0.0" Y="0.0" Z="185.0" XR="0.0" YR="0.0" ZR="0.0" Mass="0.9" MX="0.0" MY="0.0" MZ="71.0"/>
    <Tool ID="7" Name="gripper_bottom_cover" X="0.0" Y="0.0" Z="185.0" XR="0.0" YR="0.0" ZR="0.0" Mass="0.9" MX="0.0" MY="0.0" MZ="71.0"/>
    <Tool ID="4" Name="gripper_parallel" X="0.0" Y="0.0" Z="0.0" XR="185.0" YR="0.0" ZR="0.0" Mass="0.9" MX="0.0" MY="0.0" MZ="71.0"/>
    <Tool ID="3" Name="schunk_hand" X="0.0" Y="0.0" Z="127.0" XR="0.0" YR="0.0" ZR="0.0" Mass="2.2" MX="0.0" MY="0.0" MZ="125.0"/>
    <Tool ID="5" Name="robotiq" X="0.0" Y="0.0" Z="260.0" XR="0.0" YR="0.0" ZR="0.0" Mass="2.7" MX="0.0" MY="0.0" MZ="125.0"/>
  </ROBOT>
  */
  LIBRARY_API bool CrpiRobotXml::encode (char *line)
  {
    std::stringstream strm;
    vector<CrpiToolDef>::iterator titer;
    vector<string>::iterator niter;
    vector<Math::matrix*>::iterator miter;
    vector<robotPose>::iterator piter;

    if (params_ != NULL)
    {
      strm << "<ROBOT>\n <TCP_IP Address=\"" << params_->tcp_ip_addr << "\" Port=\"" << params_->tcp_ip_port
           << "\" Client=\"" << (params_->tcp_ip_client ? "true" : "false") << "\"/>\n  <ComType Val=\""
           << (params_->use_serial ? "SERIAL" : "TCP_IP") << "\"/>\n  <Mounting X=\"" << params_->mounting->x
           << "\" Y=\"" << params_->mounting->y << "\" Z=\"" << params_->mounting->z << "\" XR=\""
           << params_->mounting->xrot << "\" YR=\"" << params_->mounting->yrot << "\" ZR=\"" << params_->mounting->zrot
           << "\"/>\n  <ToWorld X=\"" << params_->toWorld->x << "\" Y=\"" << params_->toWorld->y << "\" Z=\""
           << params_->toWorld->z << "\" XR=\"" << params_->toWorld->xrot << "\" YR=\"" << params_->toWorld->yrot
           << "\" ZR=\"" << params_->toWorld->zrot << "\" M00=\"" << params_->toWorldMatrix->at(0, 0) << "\" M01=\""
           << params_->toWorldMatrix->at(0, 1) << "\" M02=\"" << params_->toWorldMatrix->at(0, 2) << "\" M03=\""
           << params_->toWorldMatrix->at(0, 3) << "\" M10=\"" << params_->toWorldMatrix->at(1, 0) << "\" M11=\""
           << params_->toWorldMatrix->at(1, 1) << "\" M12=\"" << params_->toWorldMatrix->at(1, 2) << "\" M13=\""
           << params_->toWorldMatrix->at(1, 3) << "\" M20=\"" << params_->toWorldMatrix->at(2, 0) << "\" M21=\""
           << params_->toWorldMatrix->at(2, 1) << "\" M22=\"" << params_->toWorldMatrix->at(2, 2) << "\" M23=\""
           << params_->toWorldMatrix->at(2, 3) << "\" M30=\"" << params_->toWorldMatrix->at(3, 0) << "\" M31=\""
           << params_->toWorldMatrix->at(3, 1) << "\" M32=\"" << params_->toWorldMatrix->at(3, 2) << "\" M33=\""
           << params_->toWorldMatrix->at(3, 3) << "\"/>\n";
           
      //! Encode coordinate system transformations
      niter = params_->coordSystNames.begin();
      miter = params_->toCoordSystMatrices.begin();
      piter = params_->toCoordSystPoses.begin();
      for (; niter != params_->coordSystNames.end(); ++niter, ++miter, ++piter)
      {
        strm << "<CoordSystem Name=\"" << niter->c_str() << "\" X=\"" << (*piter).x << "\" Y=\"" 
             << (*piter).y << "\" Z=\"" << (*piter).z << "\" XR=\"" << (*piter).xrot << "\" YR=\""
             << (*piter).yrot << "\" ZR=\"" << (*piter).zrot << "\" M00=\"" << (*miter)->at(0, 0) 
             << "\" M01=\"" << (*miter)->at(0, 1) << "\" M02=\"" << (*miter)->at(0, 2) << "\" M03=\""
             << (*miter)->at(0, 3) << "\" M10=\"" << (*miter)->at(1, 0) << "\" M11=\""
             << (*miter)->at(1, 1) << "\" M12=\"" << (*miter)->at(1, 2) << "\" M13=\""
             << (*miter)->at(1, 3) << "\" M20=\"" << (*miter)->at(2, 0) << "\" M21=\""
             << (*miter)->at(2, 1) << "\" M22=\"" << (*miter)->at(2, 2) << "\" M23=\""
             << (*miter)->at(2, 3) << "\" M30=\"" << (*miter)->at(3, 0) << "\" M31=\""
             << (*miter)->at(3, 1) << "\" M32=\"" << (*miter)->at(3, 2) << "\" M33=\""
             << (*miter)->at(3, 3) << "\"/>\n";
      }

      for (titer = params_->tools.begin(); titer != params_->tools.end(); ++titer)
      {
        strm << "  <Tool ID=\"" << titer->toolID << "\" Name=\"" << titer->toolName.c_str() << "\" X=\""
             << titer->TCP.x << "\" Y=\"" << titer->TCP.y << "\" Z=\"" << titer->TCP.z << "\" XR=\""
             << titer->TCP.xrot << "\" YR=\"" << titer->TCP.yrot << "\" ZR=\"" << titer->TCP.zrot << "\" Mass=\""
             << titer->mass << "\" MX=\"" << titer->centerMass.x << "\" MY=\"" << titer->centerMass.y << "\" MZ=\""
             << titer->centerMass.z << "\"/>\n";
      }           
      strm << "</ROBOT>\n";

      sprintf(line, "%s", strm.str().c_str());
    } //if (params_ != NULL)
    return true;
  }

} // XML