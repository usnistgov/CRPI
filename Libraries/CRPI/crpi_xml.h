///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Robot Interface
//  Workfile:        crpi_xml.h
//  Revision:        1.0 - 21 January, 2008
//                   2.0 - 20 February, 2015 - Subversion integrated into CRPI
//                                             library.
//
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Handler for pure XML strings.  Calls specialized XML parsers.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef CRPI_XML_H
#define CRPI_XML_H

#include <string>
#include <sstream>
#include "crpi.h"
#include "..\Math\MatrixMath.h"

namespace Xml
{
  //! @brief Parsed CRPI commands based on XML schemas
  //!
  struct CrpiXmlParams
  {
    //! @brief The CRPI command type
    //!
    CanonCommand cmd;

    //! @brief The X axis orientation vector (first column of rotation matrix)
    //!
    orientVect xaxis;

    //! @brief The Z axis orientation vector (third column of rotation matrix)
    //!
    orientVect zaxis;

    //! @brief Unknown what this value actually is.  CRCL uses it in a variety of
    //!        different ways.
    //!
    double numPositions;

    //! @brief Unknown what commands use this.
    //!
    double setting;

    //! @brief Commanded pose (X, Y, Z values only populated, orientation set using
    //!        xaxis and zaxis variables)
    //!
    robotPose *pose;

    //! @brief Measured Cartesian forces
    //!
    robotPose *forces;

    //! @brief Commanded joint values
    //!
    robotAxes *axes;

    //! @brief Measured joint torques
    //!
    robotAxes *torques;

    //! @brief Digital inputs and outputs
    //!
    robotIO *io;

    //! @brief CRCL status flag to command straight motions
    //!
    bool moveStraight;

    //! @brief Current robot status
    //!
    CanonReturn status;

    //! @brief String value for plain-text parameters
    //!
    std::string str;

    //! @brief Integer value for single-element commands
    //!
    int integer;

    //! @brief Real value for single-element commands
    //!
    double real;

    //! @brief Boolean value for single-element commands
    //!
    bool boolean;

    //! @brief Temporary storage for NxM matrices 
    //!
    Math::matrix *matrx;
    
    //! @brief Temporary storage for arbitrary-length arrays of robot poses
    //!
    vector<robotPose> poseVector;

    //! @brief Temporary storage for arbitrary-length arrays of robot axes
    //!
    vector<robotAxes> axesVector;

    //! @brief Temporary storage for arbitrary-length arrays of real numbers
    //!
    vector<double> realVector;

    //! @brief The name of the gripper currently coupled to the robot
    //!
    std::string toolName;

    int commandID;

    //! @brief The commanded value for the tool
    //!
    double toolVal;

    //! @brief Message ID iterator
    //!
    unsigned int counter;

    //! @brief Default constructor
    //!
    CrpiXmlParams()
    {
      pose = new robotPose();
      axes = new robotAxes();
      forces = new robotPose();
      torques = new robotAxes();
      io = new robotIO();
      moveStraight = true;
      setting = numPositions = 0.0f;
      counter = 0;
      matrx = new Math::matrix(4, 4);
    }

    //! @brief Default destructor
    //!
    ~CrpiXmlParams()
    {
      delete[] pose;
      delete[] axes;
      delete[] forces;
      delete[] torques;
      delete[] io;
    }
  };


  typedef enum
  {
    CompPart = 0,
    CompFixture,
    CompUnknown
  } ComponentType;



  /*
  <Components>
    <Fixture ID="F1" Name="Base Fixture"/>
    <Fixture ID="F2" Name="Spring Subassembly Fixture"/>
    <Part ID="P1" Name="Base">
      <Location>
        <X></X>
        <Y></Y>
        <Z></Z>
        <RX></RX>
        <RY></RY>
        <RZ></RZ>
      </Location>
      <Grasp></Grasp>     //JAM:  Not sure what this is...
      <File></File>
    </Part>
    ...
  </Components>
  */

  //! @brief Collaborative robot component descriptor
  //!
  struct CrpiXmlComponent
  {
    //! @brief Component ID (searchable)
    //!
    std::string ID;

    //! @brief Component name
    //!
    std::string Name;

    //! @brief Location of component in 6DoF Cartesian space
    //!
    robotPose Location;

    //! @brief File location of a CAD model
    //!
    std::string File;

    //! @brief Component type descriptor (part vs. fixture)
    //!
    ComponentType Type;

    //! @brief Default constructor
    //!
    CrpiXmlComponent()
    {
      Type = CompUnknown;
    }

    //! @brief Default destructor
    //!
    ~CrpiXmlComponent()
    {
    }

    //! @brief Determine if this component is the one specified by name
    //!
    //! @param name The name of the component being sought
    //!
    //! @return True if the names match, false otherwise
    //!
    bool isName(std::string name)
    {
      if (strcmp(Name.c_str(), name.c_str()) == 0)
      {
        return true;
      }
      return false;
    }

    //! @brief Determine if this component is the one specified by ID
    //!
    //! @param id The ID of the component being sought
    //!
    //! @return True if the IDs match, false otherwise
    //!
    bool isID(std::string id)
    {
      if (strcmp(ID.c_str(), id.c_str()) == 0)
      {
        return true;
      }
      return false;
    }
  };


  typedef enum
  {
    AgentRobot = 0,
    AgentOperator,
    AgentUnknown
  } AgentType;


  /*
  <Agents>
    <Robot ID="R1" Name="KUKA"/>
    <Robot ID="R2" Name="UR10"/>
    <Robot ID="R3" Name="UR5"/>
    <Operator ID="H1" Name="Operator 1"/>
    <Operator ID="H2" Name="Operator 2"/>
  </Agents>
  */

  //! @brief Collaborative robot component descriptor
  //!
  struct CrpiXmlAgent
  {
    //! @brief Component ID (searchable)
    //!
    std::string ID;

    //! @brief Component name
    //!
    std::string Name;

    //! @brief Component type descriptor (part vs. fixture)
    //!
    AgentType Type;

    //! @brief Default constructor
    //!
    CrpiXmlAgent()
    {
      Type = AgentUnknown;
    }

    //! @brief Default destructor
    //!
    ~CrpiXmlAgent()
    {
    }

    //! @brief Determine if this agent is the one specified by name
    //! 
    //! @param name The name of the agent being sought
    //!
    //! @return True if the names match, false otherwise
    //!
    bool isName(std::string name)
    {
      if (strcmp(Name.c_str(), name.c_str()) == 0)
      {
        return true;
      }
      return false;
    }

    //! @brief Determine if this agent is the one specified by ID
    //!
    //! @param name The ID of the agent being sought
    //!
    //! @return True if the IDs match, false otherwise
    //!
    bool isID(std::string id)
    {
      if (strcmp(ID.c_str(), id.c_str()) == 0)
      {
        return true;
      }
      return false;
    }
  };


  typedef enum
  {
    StepLocate = 0,
    StepMove,
    StepOpen,
    StepClose,
    StepInsert
  } StepType;



  /*
    <Step ID="1.2.11.2" Name="Move to Hover">
    </Step>
  */
  //! @brief Collaborative robot process step descriptor
  //!
  struct CrpiProcessStep
  {
    //! @brief The type of process-level step to take (derived from description)
    //!
    StepType Type;

    //! @brief The unique identifier for this process step
    //!
    std::string ID;

    //! @brief Description of the process step
    //!
    std::string Description;

    //! @brief Identifier of the component being moved
    //!
    std::string ComponentID;

    //! @brief Default constructor
    //!
    CrpiProcessStep()
    {
    }

    //! @brief Default destructor
    //!
    ~CrpiProcessStep()
    {
    }

    //! @brief Determine if this process step is the one specified by ID
    //!
    //! @param name The ID of the process step being sought
    //!
    //! @return True if the IDs match, false otherwise
    //!
    bool isID(std::string id)
    {
      if (strcmp(ID.c_str(), id.c_str()) == 0)
      {
        return true;
      }
      return false;
    }
  };


  /*
  <Process ID="1.2.1" Name="Acquire Base (P1)">
    <Dependency>none</Dependency>
    <Agent></Agent>
    <PrevProcess></PrevProcess>
    <Step ID="1.2.1.1" Name="Locate Base">
    </Step>
    ...
  </Process> 
  */

  //! @brief Collaborative robot process descriptor
  //!
  struct CrpiProcess
  {
    //! @brief Unique identifier for this process
    //!
    std::string ID;

    //! @brief The name/description of this process
    //!
    std::string Name;

    //! @brief The ID of the previous process
    //!
    //! @JAM:  Not sure exactly what this is used for
    //!
    std::string PrevProcess;

    //! @brief The agent to which this process has been assigned
    //!
    //! @JAM:  Should this be defined a priori?
    //!
    std::string Agent;

    //! @brief The list of process IDs upon which this process is dependent before it can be completed
    //!
    vector<std::string> Dependencies;

    //! @brief Collection of sub-steps for the completion of this process
    //!
    vector<CrpiProcessStep> Steps;

    //! @brief Default constructor
    //!
    CrpiProcess()
    {
    }

    //! @brief Default destructor
    //!
    ~CrpiProcess()
    {
      Dependencies.clear();
      Steps.clear();
    }

    //! @brief Determine if this process is the one specified by name
    //! 
    //! @param name The name of the process being sought
    //!
    //! @return True if the names match, false otherwise
    //!
    bool isName(std::string name)
    {
      if (strcmp(Name.c_str(), name.c_str()) == 0)
      {
        return true;
      }
      return false;
    }

    //! @brief Determine if this process is the one specified by ID
    //!
    //! @param name The ID of the process being sought
    //!
    //! @return True if the IDs match, false otherwise
    //!
    bool isID(std::string id)
    {
      if (strcmp(ID.c_str(), id.c_str()) == 0)
      {
        return true;
      }
      return false;
    }
  };


  /*
  <Subtask ID="1.1" Name="Spring SubAssembly (P1)">
    <Task ID="1.2.1" Name="Acquire Base (P1)">
      <Dependency>none</Dependency>
      <Agent></Agent>
      <PrevTask></PrevTask>
      <Step ID="1.2.1.1" Name="Locate Base">
      </Step>
      ...
    </Task>
    ...
  </Subtask>
  */

  //! @brief Collaborative robot subtask descriptor
  //!
  struct CrpiSubtask
  {
    //! @brief Unique identifier for this subtask
    //!
    std::string ID;

    //! @brief The name/description of this subtask
    //!
    std::string Name;

    //! @brief Collection of processes
    //!
    vector<CrpiProcess> Processes;

    //! @brief Default constructor
    //!
    CrpiSubtask()
    {
    }

    //! @brief Default destructor
    //!
    ~CrpiSubtask()
    {
      Processes.clear();
    }

    //! @brief Determine if this subtask is the one specified by name
    //! 
    //! @param name The name of the subtask being sought
    //!
    //! @return True if the names match, false otherwise
    //!
    bool isName(std::string name)
    {
      if (strcmp(Name.c_str(), name.c_str()) == 0)
      {
        return true;
      }
      return false;
    }

    //! @brief Determine if this subtask is the one specified by ID
    //!
    //! @param name The ID of the subtask being sought
    //!
    //! @return True if the IDs match, false otherwise
    //!
    bool isID(std::string id)
    {
      if (strcmp(ID.c_str(), id.c_str()) == 0)
      {
        return true;
      }
      return false;
    }
  };


  /*
  <Task ID="1" Name="Assembly Artifact">
    <Subtask ID="1.1" Name="Spring SubAssembly (P1)">
      <Task ID="1.2.1" Name="Acquire Base (P1)">
        <Dependency>none</Dependency>
        <Agent></Agent>
        <PrevTask></PrevTask>
        <Step ID="1.2.1.1" Name="Locate Base">
        </Step>
        ...
      </Task>
      ...
    </Subtask>
    ...
  </Task>
  */

  //! @brief Collaborative robot task descriptor
  //!
  struct CrpiTask
  {
    //! @brief Unique identifier for this task
    //!
    std::string ID;

    //! @brief The name/description of this task
    //!
    std::string Name;

    //! @brief Collection of sub-steps for the completion of this task
    //!
    vector<CrpiSubtask> Subtasks;

    //! @brief Default constructor
    //!
    CrpiTask()
    {
    }

    //! @brief Default destructor
    //!
    ~CrpiTask()
    {
      Subtasks.clear();
    }

    //! @brief Determine if this task is the one specified by name
    //! 
    //! @param name The name of the task being sought
    //!
    //! @return True if the names match, false otherwise
    //!
    bool isName(std::string name)
    {
      if (strcmp(Name.c_str(), name.c_str()) == 0)
      {
        return true;
      }
      return false;
    }

    //! @brief Determine if this task is the one specified by ID
    //!
    //! @param name The ID of the task being sought
    //!
    //! @return True if the IDs match, false otherwise
    //!
    bool isID(std::string id)
    {
      if (strcmp(ID.c_str(), id.c_str()) == 0)
      {
        return true;
      }
      return false;
    }
  };


  /*
   <Program ID="Program1" Name="Assembly Artifact 1" RefFrame="W1">
    <Components>
      <Fixture ID="F1" Name="Base Fixture">
      </Fixture>
      ...
      <Part ID="P1" Name="Base">
        <Location>
          <X></X>
          <Y></Y>
          <Z></Z>
          <RX></RX>
          <RY></RY>
          <RZ></RZ>
        </Location>
        <Grasp></Grasp>
        <File></File>
      </Part>
      ...
    </Components>
    <Agents>
      <Robot ID="R1" Name="KUKA">
      </Robot>
      ...
    </Agents>
    <Task ID="1" Name="Assembly Artifact">
      <Subtask ID="1.1" Name="Spring SubAssembly (P1)">
        <Process ID="1.2.1" Name="Acquire Base (P1)">
          <Dependency>none</Dependency>
          <Agent></Agent>
          <PrevTask></PrevTask>
          <Step ID="1.2.1.1" Name="Locate Base">
          </Step>
          ...
        </Process>
        ...
      </Subtask>
      ...
    </Task>
  </Program> 
  */

  //! @brief Collaborative robot program structure that is populated based on an associated
  //!        XML schema
  //!
  struct CrpiXmlProgramParams
  {
    //! @brief Unique program ID
    //!
    std::string ID;

    //! @brief Program name/description
    //!
    std::string Name;

    //! @brief The coordinate reference system in which all locations are defined
    //!
    std::string RefFrame;

    //! @brief The list of components (fixtures, parts, etc.) that are used in this program
    //!
    vector<CrpiXmlComponent> Components;

    //! @brief The list of agents that may be included in the program
    //!
    vector<CrpiXmlAgent> Agents;

    //!
    vector<CrpiProcess> Processes;

  };


  //! @ingroup Xml
  //!
  //! @brief XML parsing class based on the SAX structure
  //!
  class LIBRARY_API CrpiXml
  {
  public:

    //! @brief Constructor
    //!
    CrpiXml (CrpiXmlParams *params);

    //! @brief Default destructor
    //!
    ~CrpiXml ();

    //! @brief Parse a line of XML tagging, storing the tag value in tag and
    //!        the attribute value in attribute.  
    //!
    //! @param line The input string to be parsed
    //!
    //! @return True if parsing was successful, false otherwise
    //!
    //! The order of attributes coincides with the order of attribute tags
    //! @note Currently only has minimal error handling
    //!
    bool parse (const std::string& line);

    //! @brief Encode an XML string from an input schema
    //!
    //! @param line The output XML string
    //!
    //! @return True if encoding was successful, false otherwise
    //!
    bool encode (char* line);

  private:

    CrpiXmlParams *params_;

    bool vectoractive;
    bool matrixactive;
    bool stringactive;
    bool realactive;
    bool intactive;
    bool poseactive;
    bool axesactive;

    //! @brief Parse the text between the tag pair
    //!
    //! @param ch The string of character located between the tag pair
    //!
    //! @return True if parsing is successful
    //!
    bool characters (const std::vector<std::string>& ch);

    //! @brief Parse the first tag of a tag pair
    //!
    //! @param tagName The tag label
    //! @param attr    String of additional attributes located within the tag
    //!
    //! @return True if parsing is successful
    //!
    bool startElement (const std::string& tagName, 
                       const xmlAttributes& attr);

    //! @brief Parse the first tag of a tag pair
    //!
    //! @param tagName The tag label
    //! @param attr    String of additional attributes located within the tag
    //!
    //! @return True if parsing is successful
    //!
    bool interTagElement (const std::string& tagName, 
                          const std::vector<std::string>& attr);

    //! @brief Parse the second tag of a tag pair
    //!
    //! @param tagName The tag label
    //!
    //! @return True if parsing is successful
    //!
    bool endElement (const std::string& tagName);

  }; // CrpiXml


  //! @ingroup Xml
  //!
  //! @brief XML parsing class based on the SAX structure
  //!
  class LIBRARY_API CrclXml
  {
  public:

    //! @brief Constructor
    //!
    CrclXml (CrpiXmlParams *params);

    //! @brief Default destructor
    //!
    ~CrclXml ();

    //! @brief Parse a line of XML tagging, storing the tag value in tag and
    //!        the attribute value in attribute.  
    //!
    //! @param line The input string to be parsed
    //!
    //! @return True if parsing was successful, false otherwise
    //!
    //! The order of attributes coincides with the order of attribute tags
    //! @note Currently only has minimal error handling
    //!
    bool parse (const std::string& line);

    //! @brief Encode an XML string from an input schema
    //!
    //! @param line The output XML string
    //!
    //! @return True if encoding was successful, false otherwise
    //!
    bool encode (char* line);

  private:

    CrpiXmlParams *params_;

    bool xaxisactive;
    bool zaxisactive;

    //! @brief Parse the text between the tag pair
    //!
    //! @param ch The string of character located between the tag pair
    //!
    //! @return True if parsing is successful
    //!
    bool characters (const std::vector<std::string>& ch);

    //! @brief Parse the first tag of a tag pair
    //!
    //! @param tagName The tag label
    //! @param attr    String of additional attributes located within the tag
    //!
    //! @return True if parsing is successful
    //!
    bool startElement (const std::string& tagName, 
                       const xmlAttributes& attr);

    //! @brief Parse the first tag of a tag pair
    //!
    //! @param tagName The tag label
    //! @param attr    String of additional attributes located within the tag
    //!
    //! @return True if parsing is successful
    //!
    bool interTagElement (const std::string& tagName, 
                          const std::vector<std::string>& attr);

    //! @brief Parse the second tag of a tag pair
    //!
    //! @param tagName The tag label
    //!
    //! @return True if parsing is successful
    //!
    bool endElement (const std::string& tagName);

  }; // CrclXml



  //! @ingroup Xml
  //!
  //! @brief XML parsing of collaborative robot program representations
  //!
  class LIBRARY_API CrpiProgramXml
  {
  public:

    //! @brief Constructor
    //!
    CrpiProgramXml(CrpiXmlProgramParams *params);

    //! @brief Default destructor
    //!
    ~CrpiProgramXml();

    //! @brief Parse a line of XML tagging, storing the tag value in tag and
    //!        the attribute value in attribute.  
    //!
    //! @param line The input string to be parsed
    //!
    //! @return True if parsing was successful, false otherwise
    //!
    //! The order of attributes coincides with the order of attribute tags
    //! @note Currently only has minimal error handling
    //!
    bool parse(const std::string& line);

    //! @brief Encode an XML string from an input schema
    //!
    //! @param line The output XML string
    //!
    //! @return True if encoding was successful, false otherwise
    //!
    bool encode(char* line);

  private:

    CrpiXmlProgramParams *params_;

    //! @brief Parse the text between the tag pair
    //!
    //! @param ch The string of character located between the tag pair
    //!
    //! @return True if parsing is successful
    //!
    bool characters(const std::vector<std::string>& ch);

    //! @brief Parse the first tag of a tag pair
    //!
    //! @param tagName The tag label
    //! @param attr    String of additional attributes located within the tag
    //!
    //! @return True if parsing is successful
    //!
    bool startElement(const std::string& tagName,
                      const xmlAttributes& attr);

    //! @brief Parse the first tag of a tag pair
    //!
    //! @param tagName The tag label
    //! @param attr    String of additional attributes located within the tag
    //!
    //! @return True if parsing is successful
    //!
    bool interTagElement(const std::string& tagName,
                         const std::vector<std::string>& attr);

    //! @brief Parse the second tag of a tag pair
    //!
    //! @param tagName The tag label
    //!
    //! @return True if parsing is successful
    //!
    bool endElement(const std::string& tagName);

  }; // CrclProgramXml


} // Xml namespace

#endif