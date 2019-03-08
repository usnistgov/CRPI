///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       CRPI XML Handler
//  Workfile:        XMLInterface.cpp
//  Revision:        1.0 - 20 September, 2016
//
//  Description
//  ===========
//  CRPI XML handler application.  Creates threads for each robot specified
//  in settings.dat, each of which opens a server on a specified port that
//  takes XML commands from a remote client.
///////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fstream>
#include "crpi_robot.h"
#include "crpi_abb.h"
#include "crpi_kuka_lwr.h"
#include "crpi_universal.h"
#include "crpi_robotiq.h"
#include "ulapi.h"

//#define XMLINTERFACE_NOISY
#define XMLINTERFACE_DEBUGTEST

using namespace std;
using namespace crpi_robot;

typedef CrpiUniversal robArmType;


struct globalHandle
{
  //! @brief Whether or not to continue running this thread
  //!
  bool runThread;  
  
  //! @brief Path to the robot's CRPI configuration XML file
  string path;

  //! @brief Port to open for client connection
  //!
  int port;

  //! @brief Constructor
  //!
  //! @param armptr Pointer to the robot object (for getting feedback from the robot)
  //!
  globalHandle ()
  {
    runThread = false;
    path = string();
  }

  //! @brief Destructor
  //!
  ~globalHandle ()
  {
  }
};


//! @brief Thread method for communicating with an ABB robot
//!
//! @param param Pointer to a globalHandle object containing runtime instructions
//!
void armABBHandlerThread(void *param)
{
  globalHandle *gH = (globalHandle*)param;
  CrpiRobot<CrpiAbb> arm(gH->path.c_str());
  ulapi_integer server, client;
  bool clientConnected = false;

  crpi_timer timer;
  char buffer[2048];
  string str;
  ulapi_integer rec, sent;

  //! Create socket connection
  server = ulapi_socket_get_server_id(gH->port);
  ulapi_socket_set_blocking(server);

  while (gH->runThread)
  {
    if (!clientConnected)
    {
      cout << "Running XML Interface on port " << gH->port << " for the ABB arm" << endl;
      client = ulapi_socket_get_connection_id(server);
      ulapi_socket_set_blocking(client);
      clientConnected = true;
      cout << "Remote ABB client connected..." << endl;
    }

    while (clientConnected && gH->runThread)
    {
      rec = ulapi_socket_read(client, buffer, 2048);
      if (rec > 0)
      {
        str = buffer;
        arm.CrpiXmlHandler(str);
        arm.CrpiXmlResponse(buffer);
        sent = ulapi_socket_write(client, buffer, strlen(buffer));
      }
      else
      {
        timer.waitUntil(5);
      }
    } // while (clientConnected && gH->runThread)
  } // while (gH->runThread)

  gH = NULL;
  return;
}


//! @brief Thread method for communicating with a UR robot
//!
//! @param param Pointer to a globalHandle object containing runtime instructions
//!
void armUniversalHandlerThread(void *param)
{
  globalHandle *gH = (globalHandle*)param;
  cout << "Creating robot using " << gH->path.c_str() << endl;
  CrpiRobot<CrpiUniversal> arm(gH->path.c_str());
  cout << "Robot Created" << endl;
  ulapi_integer server, client;
  bool clientConnected = false;

  crpi_timer timer;
  char buffer[2048];
  string str;
  ulapi_integer rec, sent;

  //! Create socket connection
  server = ulapi_socket_get_server_id(gH->port);
  ulapi_socket_set_blocking(server);

  while (gH->runThread)
  {
    if (!clientConnected)
    {
      cout << "Running XML Interface on port " << gH->port << " for the Universal arm" << endl;
      client = ulapi_socket_get_connection_id(server);
      ulapi_socket_set_blocking(client);
      clientConnected = true;
      cout << "Remote Universal client connected..." << endl;
    }

    while (clientConnected && gH->runThread)
    {
      rec = ulapi_socket_read(client, buffer, 2048);
      if (rec > 0)
      {
        str = buffer;
        arm.CrpiXmlHandler(str);

        arm.CrpiXmlResponse(buffer);
        sent = ulapi_socket_write(client, buffer, strlen(buffer));
      }
      else
      {
        timer.waitUntil(5);
      }
    } // while (clientConnected && gH->runThread)
  } // while (gH->runThread)

  gH = NULL;
  return;
}


//! @brief Thread method for communicating with a Kuka robot
//!
//! @param param Pointer to a globalHandle object containing runtime instructions
//!
void armKukaHandlerThread(void *param)
{
  globalHandle *gH = (globalHandle*)param;
  CrpiRobot<CrpiKukaLWR> arm(gH->path.c_str());
  ulapi_integer server, client;
  bool clientConnected = false;

  crpi_timer timer;
  char buffer[2048];
  string str;
  ulapi_integer rec, sent;

  //! Create socket connection
  server = ulapi_socket_get_server_id(gH->port);
  ulapi_socket_set_blocking(server);

  while (gH->runThread)
  {
    if (!clientConnected)
    {
      cout << "Running XML Interface on port " << gH->port << " for the KUKA arm" << endl;
      client = ulapi_socket_get_connection_id(server);
      ulapi_socket_set_blocking(client);
      clientConnected = true;
      cout << "Remote KUKA client connected..." << endl;
    }

    while (clientConnected && gH->runThread)
    {
      rec = ulapi_socket_read(client, buffer, 2048);
      if (rec > 0)
      {
        str = buffer;
        arm.CrpiXmlHandler(str);
        arm.CrpiXmlResponse(buffer);
        sent = ulapi_socket_write(client, buffer, strlen(buffer));
      }
      else
      {
        timer.waitUntil(5);
      }
    } // while (clientConnected && gH->runThread)
  } // while (gH->runThread)

  gH = NULL;
  return;
}


//! @brief Thread method for communicating with a Robotiq robot hand
//!
//! @param param Pointer to a globalHandle object containing runtime instructions
//!
void armRobotiqHandlerThread(void *param)
{
  globalHandle *gH = (globalHandle*)param;
  CrpiRobot<CrpiRobotiq> arm(gH->path.c_str());
  ulapi_integer server, client;
  bool clientConnected = false;

  crpi_timer timer;
  char buffer[2048];
  string str;
  ulapi_integer rec, sent;

  //! Create socket connection
  server = ulapi_socket_get_server_id(gH->port);
  ulapi_socket_set_blocking(server);

  while (gH->runThread)
  {
    if (!clientConnected)
    {
      cout << "Running XML Interface on port " << gH->port << " for the Robotiq arm" << endl;
      client = ulapi_socket_get_connection_id(server);
      ulapi_socket_set_blocking(client);
      clientConnected = true;
      cout << "Remote RObotiq client connected..." << endl;
    }

    while (clientConnected && gH->runThread)
    {
      rec = ulapi_socket_read(client, buffer, 2048);
      if (rec > 0)
      {
        str = buffer;
        arm.CrpiXmlHandler(str);
        arm.CrpiXmlResponse(buffer);
        sent = ulapi_socket_write(client, buffer, strlen(buffer));
      }
      else
      {
        timer.waitUntil(5);
      }
    } // while (clientConnected && gH->runThread)
  } // while (gH->runThread)

  gH = NULL;
  return;
}


//! @brief Example client thread that sends simple up/down commands and gets feedback
//!
//! @param param Pointer to a globalHandle object containing runtime instructions
//!
void nodDemo(void *param)
{
  globalHandle *gH = (globalHandle*)param;
  ulapi_integer server, got;
  char inbuffer[REQUEST_MSG_SIZE];

  server = ulapi_socket_get_client_id(30012, "127.0.0.1");
  if (server < 0)
  {
    cout << "could not connect sample client" << endl;
    return;
  }
  else
  {
    cout << "connected sample client" << endl;
  }

  sprintf(inbuffer, "<CRPICommand type=\"Couple\"><String Value = \"flange_ring\"/></CRPICommand>");
  ulapi_socket_write(server, inbuffer, strlen(inbuffer));
  Sleep(1000);

  bool on = true;
  while (true)
  {
    sprintf(inbuffer, "<CRPICommand type=\"SetRobotDO\"><Int Value = \"0\">/<Boolean Value = \"%s\"/></CRPICommand>", (on ? "true" : "false"));
    //cout << "sending " << inbuffer << endl;
    ulapi_socket_write(server, inbuffer, strlen(inbuffer));

    got = ulapi_socket_read(server, inbuffer, REQUEST_MSG_SIZE);
    inbuffer[got] = '\0';
    cout << got << " " << inbuffer << endl;
    Sleep(1000);
    on = !on;
  } // while (gH->runThread)

  gH = NULL;
  return;
}


//! @brief Main program method
//!
int main ()			// FMP
{
  ifstream infile("xmlsettings.dat");
  string robot, path;
  int port;
  vector<globalHandle> handles;
  vector<void*> armTasks;
  void *armtask;
  globalHandle handle;

#ifdef XMLINTERFACE_DEBUGTEST
  globalHandle demoHandle;
  void *demoTask;
#endif


  while (infile >> robot)
  {
    infile >> path >> port;

    //! Start the threads based on the information parsed from settings.dat
    cout << "Starting server for " << robot << " on port " << port << endl;
    if (robot == "ABB_IRB_14000")
    {
      handle.runThread = true;
      handle.path = path;
      handle.port = port;

      armtask = ulapi_task_new();
      //! Start new ABB thread
      ulapi_task_start((ulapi_task_struct*)armtask, armABBHandlerThread, &handle, ulapi_prio_lowest(), 0);
      handles.push_back(handle);
      armTasks.push_back(armtask);
    }
    else if (robot == "UNIVERSAL")
    {
      handle.runThread = true;
      handle.path = path;
      handle.port = port;

      armtask = ulapi_task_new();
      //! Start new Universal Robot thread
      ulapi_task_start((ulapi_task_struct*)armtask, armUniversalHandlerThread, &handle, ulapi_prio_lowest(), 0);
      handles.push_back(handle);
      armTasks.push_back(armtask);
    }
    else if (robot == "KUKA_LWR")
    {
      handle.runThread = true;
      handle.path = path;
      handle.port = port;

      armtask = ulapi_task_new();
      //! Start new Kuka thread
      ulapi_task_start((ulapi_task_struct*)armtask, armKukaHandlerThread, &handle, ulapi_prio_lowest(), 0);
      handles.push_back(handle);
      armTasks.push_back(armtask);
    }
    else if (robot == "ROBOTIQ")
    {
      handle.runThread = true;
      handle.path = path;
      handle.port = port;

      armtask = ulapi_task_new();
      //! Start new Kuka thread
      ulapi_task_start((ulapi_task_struct*)armtask, armRobotiqHandlerThread, &handle, ulapi_prio_lowest(), 0);
      handles.push_back(handle);
      armTasks.push_back(armtask);
    }
    else
    {
      cout << "Error in xmlsettings.dat.  Unknown robot type: " << robot << endl;
    }
    Sleep(1000);
  } // while (infile >> robot)


#ifdef XMLINTERFACE_DEBUGTEST
  Sleep(5000);
  demoTask = ulapi_task_new();
  ulapi_task_start((ulapi_task_struct*)demoTask, nodDemo, &demoHandle, ulapi_prio_lowest(), 0);

#endif


  crpi_timer timer;
  while (true)
  {
    //! Put any additional logic here
    
    //! Throttle back evaluation of the main thread to keep from starving the other threads
    timer.waitUntil(500);
  } // while (true)

  vector<globalHandle>::iterator ghIter;
  vector<void*>::iterator taskIter;

  //! Garbage collection.  Stop the threads.
  for (ghIter = handles.begin(), taskIter = armTasks.begin(); ghIter != handles.end(); ++ghIter, ++taskIter)
  {
    ghIter->runThread = false;
    timer.waitUntil(100);
    ulapi_task_stop((ulapi_task_struct*)*taskIter);
  }

  return 0;			// FMP
}
