///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Behavior Server
//  Subsystem:       
//  Workfile:        types.h
//  Revision:        1.0 - 6 Apr, 2006
//                   1.1 - 2 July, 2007 : Modified for use on the cell phone
//                                        assembly configuration
//                   2.0 - 1 Sep, 2008  : Generalized for multiple projects
//                   2.1 - 24 May, 2010 : Moved common data types from GA
//                                        library to here
//  Author:          J. Marvel
//
//  Description
//  ===========
//  System-wide type & structure definitions
//
///////////////////////////////////////////////////////////////////////////////

#ifndef TYPES_H
#define TYPES_H

#include <vector>
#include <string>
#include <math.h>
#if defined(_MSC_VER)
#include <io.h>
#elif defined(__GNUC__)
#include <sys/io.h>
typedef int SOCKET;
#endif
#include "portable.h"

///////////////////////////////////////////////////////////////////////////////
/**************************** data structures ********************************/
///////////////////////////////////////////////////////////////////////////////




//! @brief State setting variables for the server
//!
struct networkSettings
{
  //! @brief Whether we are currently connected
  //!
  bool connected;

  //! @brief Whether the server is currently running
  //!
  bool globalRunServer;

  //! @brief Whether a disconnect request has been issued
  //!
  bool disconnectRequest;

  //! @brief Internal debugging string
  //!
  char globalString[1024];

  //! @brief Internal debugging socket string
  //!
  char sockString[1024];

  //! @brief Run process flag
  //!
  unsigned short runFlag;

  //! @brief Whether or not to commit the GSI logs to disk
  //!
  bool log;

  //! @brief Default settings constructor
  //!
  networkSettings  () : 
    connected(false),
    globalRunServer(false),
    log(false),
    disconnectRequest(false)
  {
  }
};


//! @brief Connection information for networking
//!
//! Formerly known as socketStruct
//!
struct LIBRARY_API networkStruct
{
  //! @brief Whether or not this structure has been populated with data
  //!
  bool defined;

  //! @brief Client's socket file descriptor
  //!
  SOCKET cFd;

  //! @brief Server's socket file descriptor
  //!
  SOCKET sFd;

  //! @brief Current computer's IP address
  //!
  char* serverAddress;

  //! @brief Attached computer's IP address
  //!
  char* address;

  //! @brief Client's port number
  //!
  unsigned int cP;

  //! @brief Server's port number
  //!
  unsigned int sP;

  //! @brief Whether we are currently connected (formerly in networkSettings)
  //!
  bool connected;

  //! @brief Whether the server file descriptor is currently bound
  //!
  bool bound;

  //! @brief Whether the socket is a server
  //!
  bool server;

  int bytesRead;
  int bytesSent;

  //! @brief Default constructor
  //!
  networkStruct() :
    defined(false),
    connected(false),
    bound(false),
    server(false),
    bytesRead(0),
    bytesSent(0)
    {
      serverAddress = new char[16];
    }
};



#endif
