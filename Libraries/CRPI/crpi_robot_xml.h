///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       XML
//  Workfile:        crpi_robot_xml.h
//  Revision:        1.0 - 17 June, 2015
//
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Handler for pure XML strings of CRPI robot configurations and commands
//
///////////////////////////////////////////////////////////////////////////////

#ifndef CRPI_ROBOT_XML_H
#define CRPI_ROBOT_XML_H

#include <vector>
#include <string>
#include "crpi.h"

namespace Xml
{
  //! @ingroup Xml
  //!
  //! @brief XML parsing class based on the SAX structure
  //!
  class LIBRARY_API CrpiRobotXml
  {
  public:

    //! @brief Constructor
    //!
    CrpiRobotXml (CrpiRobotParams *params);

    //! @brief Default destructor
    //!
    ~CrpiRobotXml ();

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

    CrpiRobotParams *params_;
    CrpiToolDef *toolTmp;

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

  }; // CrpiRobotXml
} // Xml namespace

#endif