///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Serial Communications
//  Workfile:        serial.cpp
//  Revision:        1.0 - 25 March, 2014
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Network connectivity - Serial class.  Note that the get and send functions
//  require a serialStruct object passed as an argument.  This way we can have
//  a single network object sending data to multiple destinations simply by
//  passing different serial connection parameters.
//
///////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include "serial.h"

//#define NOISY

namespace Network
{
  //! *************************************************************************
  //!                       SERIAL COMMUNICATION METHODS
  //! *************************************************************************
  LIBRARY_API serial::serial (networkSettings *settings)
  {
  }

  LIBRARY_API serial::serial ()
  {
    //logger_ = NULL;
  }


  LIBRARY_API serial::~serial ()
  {
    //settings_ = NULL;
  }


  //! Note:  this function blocks
  LIBRARY_API bool serial::getData (char *buffer, serialStruct serialData, int bytes)
  {
    DWORD dwBytes;
#if defined(_MSC_VER)
    do
    {
      if (bytes < 0)
      {
        ReadFile (serialData.serial, buffer, REQUEST_MSG_SIZE, &dwBytes, NULL);
      }
      else
      {
        ReadFile (serialData.serial, buffer, bytes, &dwBytes, NULL);
      }
#ifdef NOISY
      //logger_->log (buffer);
#endif
    } while (dwBytes == 0);
#elif defined(__GNUC__)
	read(serialData.serial, buffer, bytes);
#endif
    return true;
  }


  LIBRARY_API bool serial::getRawBytes (void *bytes, int &numBytes)
  {
    return false;
  }


  LIBRARY_API bool serial::dataWaiting (serialStruct &conn)
  {
    return false;
  }


  LIBRARY_API bool serial::sendData (const char * buffer, struct serialStruct serialData)
  {
    DWORD dwBytes;
    int resp;

    if (serialData.connected)
    {
#ifdef NOISY
      //logger_->log (buffer);
#endif
#if defined(_MSC_VER)
      resp = WriteFile (serialData.serial, buffer, strlen(buffer) + 1, &dwBytes, NULL);
#elif defined(__GNUC__)
		resp = write(serialData.serial, buffer, strlen(buffer) + 1);
#endif
    }
    else
    {
     // logger_->error ("serial::sendData Not connected");
      return false;
    }

    if (resp == ERROR_F)
    {
      return false;
    }

    return true;
  }


  LIBRARY_API bool serial::sendRawBytes (void *bytes, int numBytes)
  {
    return false;
  }


  LIBRARY_API bool serial::create (struct serialStruct& serialData)
  {
    return true;
  } 


  LIBRARY_API bool serial::attach (serialStruct &serialData)
  {

    if (serialData.connected)
    {
      //! We're already connected on this port
      return false;
    }
#if defined(_MSC_VER)
    serialData.serial = CreateFile (serialData.COMChannel,
                                    GENERIC_READ | GENERIC_WRITE,
                                    0,
                                    NULL,
                                    OPEN_EXISTING,
                                    FILE_ATTRIBUTE_NORMAL,
                                    0);

	//! Initialize serial port
    if (serialData.serial == INVALID_HANDLE_VALUE)
    {
      return false;
    }

    serialData.dcb.DCBlength = sizeof (DCB);
    GetCommState (serialData.serial, &serialData.dcb);
    serialData.dcb.BaudRate = serialData.BaudRate;
    serialData.dcb.ByteSize = 8;
    serialData.dcb.Parity = (serialData.evenParity ? EVENPARITY : ODDPARITY);
    serialData.dcb.StopBits = (serialData.stopBits == 1 ? ONESTOPBIT : TWOSTOPBITS);

    //! Initialize COM port
    if (!SetCommState (serialData.serial, &serialData.dcb))
    {
      return false;
    }

    //! Set up COM port timeouts
    COMMTIMEOUTS CommTimeouts;
    GetCommTimeouts (serialData.serial, &CommTimeouts);
    CommTimeouts.ReadIntervalTimeout = 50;  
    CommTimeouts.ReadTotalTimeoutMultiplier = 50;  
    CommTimeouts.ReadTotalTimeoutConstant = 50;    
    CommTimeouts.WriteTotalTimeoutMultiplier = 50;  
    CommTimeouts.WriteTotalTimeoutConstant = 50;

    //! Check if timeouts setup is valid
    if (!SetCommTimeouts (serialData.serial, &CommTimeouts))
    {
      return false;
    }
#elif defined(__GNUC__)
	//defining with readwrite, not controlling terminal, and no delay
	serialData.serial = open(serialData.COMChannel, O_RDWR | O_NOCTTY | O_NDELAY);
	if (serialData.serial == -1) 
	{
		//! port could not be opened
		perror("open_port: unable to open port");
		return false;
	}
	else 
	{
		//Initializing port
		fcntl(serialData.serial, F_SETFL, 0);
	}
#endif
    serialData.connected = true;
    return true;
  }


  LIBRARY_API void serial::closeConnection (struct serialStruct& serialData)
  {
#if defined(_MSC_VER)
    CloseHandle (serialData.serial);
#elif defined(__GNUC__)
	close(serialData.serial);
#endif
	serialData.connected = false;
  }

}