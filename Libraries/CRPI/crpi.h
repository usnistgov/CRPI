///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Robot Interface
//  Workfile:        crpi.h
//  Revision:        1.0 - 13 March, 2014
//                   2.0 - 12 March, 2015 - Conversion from CRCL to CRPI
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Central Robot Programming Interface type definitions and declarations.
//
///////////////////////////////////////////////////////////////////////////////


#ifndef CRPI_H_
#define CRPI_H_

#include "ulapi.h"
#include <math.h>
#include <vector>
#include <string>
#include <time.h>
#include "Math/MatrixMath.h"
#include "Math/VectorMath.h"
#include "portable.h"

#ifndef WIN32
#include <unistd.h>
#endif

#ifdef WIN32

#if _MSC_VER>=1900
#  define STDC99
#endif

#else

#ifndef LIBRARY_API		/* FMP */
#define LIBRARY_API
#endif

#endif

#pragma warning (disable: 4996) 

//! @brief Command typedefs for message handling.
//!
//! @note Some commands specified are not part of the CRPI, but are instead part of the
//!       CRCL (and vice versa).  They are included here for convenient conversion from
//!       CRCL commands
//!
typedef enum
{
  CmdApplyCartesianForceTorque = 0,
  CmdApplyJointTorque,
  CmdCouple,
  CmdDecouple,                  //! Not a CRPI command
  CmdDwell,                     //! Not a CRPI command
  CmdEndCanon,                  //! Not a CRPI command
  CmdToWorldMatrix,
  CmdToSystemMatrix,
  CmdFromSystem,
  CmdFromWorld,
  CmdGetRobotAxes,
  CmdGetRobotForces,
  CmdGetRobotIO,
  CmdGetRobotPose,
  CmdGetRobotSpeed,
  CmdGetRobotTorques,
  CmdInitCanon,                 //! Not a CRPI command
  CmdMessage,
  CmdMoveAttractor,
  CmdMoveStraightTo,
  CmdMoveThroughTo,
  CmdMoveTo,
  CmdMoveToAxisTarget,
  CmdRunProgram,
  CmdSaveConfig,
  CmdSetAbsoluteAcceleration,
  CmdSetAbsoluteSpeed,
  CmdSetAngleUnits,
  CmdSetAxialSpeeds,
  CmdSetAxialUnits,
  CmdSetEndPoseTolerance,
  CmdSetIntermediatePoseTolerance,
  CmdSetLengthUnits,
  CmdSetParameter,
  CmdSetRelativeAcceleration,
  CmdSetRelativeSpeed,
  CmdSetRobotDO,
  CmdSetRobotIO,
  CmdSetTool,
  CmdStopMotion,
  CmdToSystem,
  CmdToWorld,
  CmdUpdateSystemTransform,
  CmdUpdateWorldTransform
} CanonCommand;

typedef enum
{
  CANON_SUCCESS = 0,
  CANON_FAILURE,
  CANON_REJECT,
  CANON_RUNNING
} CanonReturn;

typedef enum
{
  METER = 0,
  MM,
  INCH
} CanonLengthUnit;

typedef enum
{
  RADIAN = 0,
  DEGREE
} CanonAngleUnit;

typedef enum
{
  PRISMATIC = 0,
  REVOLUTE
} CanonJointType;		// FMP

//! @brief Vector representation of an axis of rotation
//!
struct orientVect
{
  double i;
  double j;
  double k;

  //! @brief Orientation assignment function
  //!
  //! @param source An existing orientation vector that will be used to
  //!               populate this orientation vector instance
  //!
  orientVect & operator=(const orientVect &source)
  {
    if (this != &source)
    {
      i = source.i;
      j = source.j;
      k = source.k;
    }
    return *this;
  }

  //! @brief Orientation adjustment function:  Addition
  //!
  //! @param pB An existing orientation vector that will be added to the
  //!           current orientation vector
  //!
  orientVect operator+(const orientVect &pB)
  {
    orientVect pC;
    pC.i = i + pB.i;
    pC.j = j + pB.j;
    pC.k = k + pB.k;
    return pC;
  }

  //! @brief Orienatation adjustment function:  Subtraction
  //!
  //! @param pB An existing orientation vector that will be added to the
  //!           current orientation vector
  //!
  orientVect operator-(const orientVect &pB)
  {
    orientVect pC;
    pC.i = i - pB.i;
    pC.j = j - pB.j;
    pC.k = k - pB.k;
    return pC;
  }
};

//! @brief Cartesian pose of an object
//!
struct robotPose
{
  //! @brief X axis location
  //!
  double x;

  //! @brief Y axis location
  //!
  double y;

  //! @brief Z axis location
  //!
  double z;

  //! @brief X axis rotation
  //!
  double xrot;

  //! @brief Y axis rotation
  //!
  double yrot;

  //! @brief Z axis rotation
  //!
  double zrot;

  //! @brief Redundancy parameter describing the orientation of axes to prevent ambiguity
  //!
  //! @note Used primarily by KUKA systems
  //! @note With KUKA robots, bit 0 specifies the position of the interection of wrist axes
  //!       (1 in overhead, 0 in basic area), bit 1 specifies the position of axis 3
  //!       (1 A3 >= 0, 0 A3 < 0), bit 2 specifies the position of axis 5 (1 A5 > 0, 0
  //!       A5 <= 0), bit 3 is always 0, and bit 5 specifies whether the point was taught
  //!       using an absolutely accurate robot (0 false, 1 true)
  //!
  int status;

  //! @brief Bitwise redundancy parameter describing the sign of the axes angles' values
  //!
  //! @note Used primarily by KUKA systems
  //! @note For an N DoF robot, the little-endian bits specify axes in order
  //!
  int turns;

  //! @brief Default constructor
  //!
  robotPose()
  {
    x = y = z = xrot = yrot = zrot = 0;
    status = turns = -1;
  }

  //! @brief Copy constructor
  //!
  robotPose (const robotPose& source)
  {
    x = source.x;
    y = source.y;
    z = source.z;
    xrot = source.xrot;
    yrot = source.yrot;
    zrot = source.zrot;
    status = source.status;
    turns = source.turns;
  }

  //! @brief Pose assignment function
  //!
  //! @param source An existing pose that will be used to populate this pose
  //!               instance
  //!
  robotPose & operator=(const robotPose &source)
  {
    if (this != &source)
    {
      x = source.x;
      y = source.y;
      z = source.z;
      xrot = source.xrot;
      yrot = source.yrot;
      zrot = source.zrot;
      status = source.status;
      turns = source.turns;
    }
    return *this;
  }


  //! @brief Pose assignment function
  //!
  //! @param source An existing pose that will be used to populate this pose
  //!               instance
  //!
  robotPose & operator=(const Math::pose &source)
  {
    x = source.x;
    y = source.y;
    z = source.z;
    xrot = source.xr;
    yrot = source.yr;
    zrot = source.zr;
    status = turns = 0;
    return *this;
  }

  //! @brief Convert to a math library pose object
  //!
  //! @return A Math::pose object containing the robotPose information sans status and turns
  //!
  Math::pose pose()
  {
    Math::pose temp;
    temp.x = x;
    temp.y = y;
    temp.z = z;
    temp.xr = xrot;
    temp.yr = yrot;
    temp.zr = zrot;
    return temp;
  }

  //! @brief Pose adjustment function:  Addition
  //!
  //! @param pB An existing pose that will be added to the current pose
  //!
  robotPose operator+(const robotPose &pB)
  {
    robotPose pC;
    pC.x = x + pB.x;
    pC.y = y + pB.y;
    pC.z = z + pB.z;
    pC.xrot = xrot + pB.xrot;
    pC.yrot = yrot + pB.yrot;
    pC.zrot = zrot + pB.zrot;
    pC.status = status;
    pC.turns = turns;
    return pC;
  }

  //! @brief Pose adjustment function:  Subtraction
  //!
  //! @param pB An existing pose that will be added to the current pose
  //!
  robotPose operator-(const robotPose &pB)
  {
    robotPose pC;
    pC.x = x - pB.x;
    pC.y = y - pB.y;
    pC.z = z - pB.z;
    pC.xrot = xrot - pB.xrot;
    pC.yrot = yrot - pB.yrot;
    pC.zrot = zrot - pB.zrot;
    pC.status = status;
    pC.turns = turns;
    return pC;
  }

  //! @brief Pose adjustment function:  Scaling
  //!
  //! @param div Scalar divisor by which all pose elements are scaled
  //!
  robotPose operator/(const double div)
  {
    robotPose pC;
    pC.x = x / div;
    pC.y = y / div;
    pC.z = z / div;
    pC.xrot = xrot / div;
    pC.yrot = yrot / div;
    pC.zrot = zrot / div;
    pC.status = status;
    pC.turns = turns;
    return pC;
  }

  //! @brief Pose adjustment function:  Scaling
  //!
  //! @param div Scalar divisor by which all pose elements are scaled
  //!
  robotPose operator*(const double mult)
  {
    robotPose pC;
    pC.x = x * mult;
    pC.y = y * mult;
    pC.z = z * mult;
    pC.xrot = xrot * mult;
    pC.yrot = yrot * mult;
    pC.zrot = zrot * mult;
    pC.status = status;
    pC.turns = turns;
    return pC;
  }

  //! @brief Calculate the Euclidean distance between two pose locations
  //!
  //! @param pB The target pose to which this pose is compared
  //!
  //! @return The L2 norm (distance in Euclidean unit space) of the two pose locations
  //!
  double distance(robotPose &pB)
  {
    return sqrt(((x-pB.x)*(x-pB.x))+((y-pB.y)*(y-pB.y))+((z-pB.z)*(z-pB.z)));
  }

  //! @brief Calculate the rotational distance between two pose orientations
  //!
  //! @param pB The target pose to which this pose is compared
  //!
  //! @return The L2 norm of the two pose orientations
  //!
  double distance_rot(robotPose &pB)
  {
    return sqrt(((xrot-pB.xrot)*(xrot-pB.xrot))+((yrot-pB.yrot)*(yrot-pB.yrot))+((zrot-pB.zrot)*(zrot-pB.zrot)));
  }

  //! @brief Display the value of this pose on the screen
  //!
  void print()
  {
    printf ("(%f, %f, %f, %f, %f, %f, %d, %d)\n", x, y, z, xrot, yrot, zrot, status, turns);
  }
};

#define CRPI_AXES_MAX 16

struct robotAxes
{
  //! @brief Set of axis values
  //!
  //double *axis;
  vector<double> axis;

  //! @brief Size of axis array
  //!
  int axes;

  //! @brief Default constructor
  //!
  robotAxes()
  {
    axis.clear();
    //axis = new double[CRPI_AXES_MAX];
    axis.resize(CRPI_AXES_MAX);
    for (int i = 0; i < CRPI_AXES_MAX; ++i)
    {
      //axis[i] = 0.0f;
      axis.at(i) = 0.0;
    }
    axes = axis.size();
    //axes = CRPI_AXES_MAX;
  }

  //! @brief Size-specifying constructor
  //!
  //! @param size The number of axes for this robot
  //!
  robotAxes(int size)
  {
    axis.clear();
    axis.resize(size);
    //axis = new double[size];
    for (int i = 0; i < size; ++i)
    {
      //axis[i] = 0.0f;
      axis.at(i) = 0.0;
    }
    axes = size;
  }

  //! @brief Default destructor
  //!
  ~robotAxes()
  {
    axis.clear();
    //if (axes > 0 && axis != NULL)
    //{
    //  delete axis;
    //}
    //axis = NULL;
    axes = 0;
  }

  //! @brief Axes assignment function
  //!
  //! @param source An existing axes set that will be used to populate this set
  //!               instance
  //!
  robotAxes & operator=(const robotAxes &source)
  {
    if (this != &source)
    {
      /*
      delete[] axis;
      axis = new double[source.axes];
      for (int i = 0; i < source.axes; ++i)
      {
        axis[i] = source.axis[i];
      }
      axes = source.axes;
      */
      axis.clear();
      for (int i = 0; i < source.axes; ++i)
      {
        axis.push_back(source.axis.at(i));
      }
      axes = source.axes;
    }
    return *this;
  }

  //! @brief Display the axes values on the screen
  //!
  void print()
  {
    for (int i = 0; i < axes; ++i) {
      printf("%f", axis.at(i));
      if (i < (axes - 1)) {
        printf(", ");
      }
    }
    printf("\n");
  }

  //! @brief Calculate the distance between two axis vectors
  //!
  //! @param target The specified axis configuration to which this collection of axes is compared
  //!
  //! @return The L2 norm of the two axis vectors
  //!
  double distance(robotAxes &target)
  {
    double dist = 0.0f;
    int i;

    if (axes != target.axes)
    {
      return -1.0f;
    }

    for (i = 0; i < axes; ++i)
    {
      dist += ((axis.at(i) - target.axis.at(i)) * (axis.at(i) - target.axis.at(i)));
    }
    return sqrt(dist);
  }

  //! @brief Calculate the average magnitude error between two axis vectors
  //!
  //! @param target The specified axis configuration to which this collection of axes is compared
  //!
  //! @return The average magnitude error of all axis values
  //!
  double error(robotAxes &target)
  {
    double err = 0.0f;
    int i;

    if (axes != target.axes)
    {
      return -1.0f;
    }

    for (i = 0; i < axes; ++i)
    {
      err += fabs(axis.at(i) - target.axis.at(i));
    }
    return err / axes;
  }
};


#define CRPI_IO_MAX 16

struct robotIO
{
  //! @brief Set of digital I/O values
  //!
  bool *dio;

  //! @brief Set of analog I/O values
  //!
  double *aio;

  //! @brief Number of DI/O values defined
  //!
  int ndio;

  //! @breif Number of AI/O values defined
  //!
  int naio;

  //! @brief Default constructor
  //!
  robotIO ()
  {
    dio = new bool[CRPI_IO_MAX];
    aio = new double[CRPI_IO_MAX];
    for (int i = 0; i < CRPI_IO_MAX; ++i)
    {
      dio[i] = false;
      aio[i] = 0.0f;
    }
    ndio = naio = CRPI_IO_MAX;
  }

  //! @brief Size-specifying constructor
  //!
  //! @param diosize The number of DIO signals
  //! @param aiosize The number of AIO signals
  //!
  robotIO(int diosize, int aiosize)
  {
    dio = new bool[diosize];
    aio = new double[aiosize];
    for (int i = 0; i < diosize; ++i)
    {
      dio[i] = false;
    }
    for (int i = 0; i < aiosize; ++i)
    {
      aio[i] = 0.0f;
    }
    ndio = diosize;
    naio = aiosize;
  }

  //! @brief Default destructor
  //!
  ~robotIO()
  {
    delete[] dio;
    delete[] aio;
  }

  //! @brief Assignment function
  //!
  //! @param source An existing IO structur that will be used to populate this
  //!               instance
  //!
  robotIO & operator=(const robotIO &source)
  {
    if (this != &source)
    {
      delete[] dio;
      delete[] aio;

      dio = new bool[source.ndio];
      aio = new double[source.naio];

      for (int i = 0; i < source.ndio; ++i)
      {
        dio[i] = source.dio[i];
      }
      for (int i = 0; i < source.naio; ++i)
      {
        aio[i] = source.aio[i];
      }
      ndio = source.ndio;
      naio = source.naio;
    }
    return *this;
  }
};

//! @brief Generic structure for passing information between threads of a multi-threaded robot object
//!
struct keepalive
{
  //! @brief Mutex for protecting shared data
  //!
  ulapi_mutex_struct *handle;

  //! @brief Terminator signal from the main thread to exit any loops still running in the target thread
  //!
  bool runThread;

  //! @brief Generic pointer to shared data
  //!
  void *rob;
};


//! @brief Inner-tag parameter attributes for XML tags (ex <tag name="val">)
//!
struct xmlAttributes
{
  //! @brief The list of parameter names
  //!
  std::vector<std::string> name;

  //! @brief The list of parameter values
  //!
  std::vector<std::string> val;
};


/*
<Robot>
<TCP_IP Address="127.0.0.1" Port="6007" Client="false"/>
<Serial Port="COM7" Rate="57600" Parity="Even" SBits="1" Handshake="None"/>
<ComType Val="Serial"/>
<Mounting X="0.0" Y="0.0" Z="0.0" XR="0.0" YR="0.0" ZR="0.0"/>
<ToWorld X="89.0" Y="89.0" Z="0.0" XR="0.0" YR="0.0" ZR="0.0"/>
<Tool ID="7" Name="gripper_top_cover" X="0.0" Y="0.0" Z="0.0" XR="0.0" YR="0.0" ZR="0.0" MX="0.0" MY="0.0" MZ="0.0/>
<Tool ID="7" Name="gripper_bottom_cover" X="0.0" Y="0.0" Z="0.0" XR="0.0" YR="0.0" ZR="0.0" MX="0.0" MY="0.0" MZ="0.0/>
<Tool ID="4" Name="gripper_parallel" X="0.0" Y="0.0" Z="0.0" XR="0.0" YR="0.0" ZR="0.0" MX="0.0" MY="0.0" MZ="0.0/>
<Tool ID="3" Name="schunk_hand" X="0.0" Y="0.0" Z="0.0" XR="0.0" YR="0.0" ZR="0.0" MX="0.0" MY="0.0" MZ="0.0/>
</Robot>
*/



//! @brief Tool definition as read in from the configuration XML file
//!
//! @note Example XML file entry:
//!       <Tool ID="7" Name="gripper_gear" X="0.0" Y="0.0" Z="0.0" XR="0.0" YR="0.0" ZR="0.0" MX="0.0" MY="0.0" MZ="0.0/>
//!
struct CrpiToolDef
{
  //! @brief The unique name given to the tool (searchable key)
  //!
  std::string toolName;

  //! @brief The unique ID given to this tool (searchable key)
  //!
  int toolID;

  //! @brief The Cartesian offset of the tool center point from the robot's tool flange (in tool frame)
  //!
  robotPose TCP;

  //! @brief The Cartesian center of the tool's center of mass (in tool frame)
  //!
  robotPose centerMass;

  //! @brief The mass (in kg) of the tool
  //!
  double mass;

  //! @brief Default constructor
  //!
  CrpiToolDef()
  {
  }

  //! @brief Default destructor
  //!
  ~CrpiToolDef()
  {
  }

  //! @brief Assignment function
  //!
  //! @param source An existing CRPI robot parameter structure used to populate this
  //!               instance
  //!
  CrpiToolDef & operator=(const CrpiToolDef &source)
  {
    toolName = source.toolName;
    toolID = source.toolID;
    TCP = source.TCP;
    mass = source.mass;
    centerMass = source.centerMass;
    return *this;
  }

  //! @brief Determine if this is a specific tool based on its name
  //!
  //! @param tarTool The name of the tool being sought
  //!
  //! @return TRUE if this tool's name matches the query string, FALSE otherwise
  //!
  bool isTool(const char* tarTool)
  {
    return (strcmp(toolName.c_str(), tarTool) == 0);
  }

  //! @brief Determine if this is a specific tool based on its ID
  //!
  //! @param tarTool The ID of the tool being sought
  //!
  //! @return TRUE if this tool's ID matches the query value, FALSE otherwise
  //!
  bool isTool(int tarTool)
  {
    return (toolID == tarTool);
  }

};

//! @brief Configuration of the robot and its controller
//!
struct CrpiRobotParams
{
  //! @brief IPv$ address of the robot motion server
  //!
  char tcp_ip_addr[16];

  //! @brief Port on which the robot controller is expecting a connection
  //!
  int tcp_ip_port;

  //! @brief Whether or not the TCP/IP connection should be established as a client (true) or a server (false)
  //!
  bool tcp_ip_client;

  //! @brief IPv4 address of the robot state observer
  //!
  char obs_tcp_ip_addr[16];

  //! @brief Port on which the robot state observer is running
  //!
  int obs_tcp_ip_port;

  //! @brief Whether or not the observer should connect as a client or open a server for clients to attach
  //!
  bool obs_tcp_ip_client;

  //! @brief Serial port descriptor
  //!
  char serial_port[5];

  //! @brief Serial connection baud rate
  //!
  int serial_rate;

  //! @brief Serial data parity
  //!
  bool serial_parity_even;

  //! @brief Serial stop bits
  //!
  int serial_sbits;

  //! @brief Serial flow control options
  //!
  char serial_handshake[10];

  //! @brief Whether or not to use serial (TCP/IP used if false)
  //!
  bool use_serial;

  //! @brief Transformation to realign the robot's coordinate system to correct for mounting
  //!
  robotPose *mounting;

  //! @brief Transformation to realign the robot's coordinate system to align with the world's
  //!
  robotPose *toWorld;

  //! @brief Whether or not a homogeneous transformation matrix was used to describe the rigid transformation
  //!        from world to the robot base origin.  Older configuration files may not have this matrix, and
  //!        should be updated automatically to include it.
  //!
  bool usedMatrix;

  //! @brief Homogeneous transformation matrix from the robot's coordinate frame to the world coordinate frame
  //!
  Math::matrix *toWorldMatrix;

  //! @brief Collection of coordinate system names
  //!
  vector<string> coordSystNames;
  
  //! @brief Collection of coordinate system transformation matrices corresponding with the coordinate system names
  //!
  vector<Math::matrix*> toCoordSystMatrices;

  //! @brief Collection of coordinate system transformation poses corresponding with the coordinate system names
  //!
  vector<robotPose> toCoordSystPoses;

  //! @brief Collection of tool definitions
  //!
  std::vector<CrpiToolDef> tools;

  // FMP
  //! @brief Path to initialization file
  char initPath[256];
  
  //! @brief Default constructor
  //!
  CrpiRobotParams()
  {
    usedMatrix = false;
    mounting = new robotPose();
    toWorld = new robotPose();
    toWorldMatrix = new Math::matrix(4, 4);
    tcp_ip_addr[0] = '\0';
    tcp_ip_port = 0;
    tcp_ip_client = false;
    obs_tcp_ip_addr[0] = '\0';
    obs_tcp_ip_port = 0;
    obs_tcp_ip_client = false;
    serial_port[0] = '\0';
    serial_rate = 0;
    serial_parity_even = false;
    serial_sbits = 0;
    serial_handshake[0] = '\0';
    use_serial = true;
    initPath[0] = '\0';		// FMP
  }

  //! @brief Assignment function
  //!
  //! @param source An existing CRPI robot parameter structure used to populate this
  //!               instance
  //!
  CrpiRobotParams & operator=(const CrpiRobotParams &source)
  {
    if (this != &source)
    {
      *mounting = *source.mounting;
      *toWorld = *source.toWorld;
      strcpy_s(tcp_ip_addr, source.tcp_ip_addr);
      tcp_ip_port = source.tcp_ip_port;
      tcp_ip_client = source.tcp_ip_client;
      strcpy_s(obs_tcp_ip_addr, source.obs_tcp_ip_addr);
      obs_tcp_ip_port = source.obs_tcp_ip_port;
      obs_tcp_ip_client = source.obs_tcp_ip_client;
      strcpy_s(serial_port, source.serial_port);
      serial_rate = source.serial_rate;
      serial_parity_even = source.serial_parity_even;
      serial_sbits = source.serial_sbits;
      strcpy_s(serial_handshake, source.serial_handshake);
      use_serial = source.use_serial;

      tools.clear();
      coordSystNames.clear();
      toCoordSystMatrices.clear();
      std::vector<CrpiToolDef>::const_iterator itr;
      for (itr = source.tools.begin(); itr != source.tools.end(); ++itr)
      {
        tools.push_back(*itr);
      }
      strcpy_s(initPath, source.initPath);
    }
    return *this;
  }

};

//! @brief Get the current system time
//!
//! @return The current system time in ms
//!
inline double getCurrentTime ()
{
  long time_ll;

#ifdef WIN32
  //! Get time from processor in ticks
  time_ll = GetTickCount ();
  //time_ll = timeGetTime ();
#elif defined VXWORKS
  pentiumTscGet64 (&time_ll);
#elif defined LINUX
  struct timespec now;
  if (clock_gettime(CLOCK_MONOTONIC, &now))
  {
    return 0.0f;
  }
  return now.tv_sec * 1000.0f + now.tv_nsec / 1000000.0f;
#endif

  //! Set the current time in ms
  return ((double)time_ll * 1.0) / 500000.0;
}


//! @brief Return a formatted string specifying the current date
//!
//! @return The date in the form YYYY-MM-DD
//!
inline const char* NumDateStr ()
{
  static char buffer[12];
  static time_t rawtime;
  static struct tm * timeinfo;

  time (&rawtime);
  timeinfo = localtime (&rawtime);

  //! YYYY-MM-DD
  strftime (buffer, 12, "%Y-%m-%d", timeinfo);

  return buffer;
};


//! @brief Return a formatted string specifying the time of day
//!
//! @return The time in the form HH-MM-SS
//!
inline const char* TimeStr ()
{
  static char buffer[12];
  static time_t rawtime;
  static struct tm * timeinfo;

  time (&rawtime);
  timeinfo = localtime (&rawtime);

  //! HH:MM:SS
  strftime (buffer, 12, "%H-%M-%S", timeinfo);

  return buffer;
};


//! @brief Timer class, includes stopwatch and alarm functionality
//!
class crpi_timer
{
public:
  //! @brief Default constructor
  //!
  crpi_timer () :
    running_(false)
  {
#ifdef WIN32
    highRes_ = (QueryPerformanceFrequency (&frequency_) ? true : false);
    timeBeginPeriod (1);
#endif
  };

  //! @brief Default destructor
  //!
  ~crpi_timer ()
  {
#ifdef WIN32			/* FMP, to match above */
    timeEndPeriod (1);
#endif
  };

  //! @brief Start the timer if it isn't already running, otherwise
  //!        let it continue to run.
  //!
  inline void start ()
  {
    if (!running_)
    {
      running_ = true;
#ifdef WIN32
    if (highRes_)
    {
      QueryPerformanceCounter (&hRstart_);
    }
    else
    {
      lRstart_ = (long)getCurrentTime ();
    }
#endif
    }
  };

  //! @brief Turn off the timer and start it again from 0
  //!
  inline void restart ()
  {
    running_ = true;
#ifdef WIN32
    if (highRes_)
    {
      QueryPerformanceCounter (&hRstart_);
    }
    else
    {
      lRstart_ = (long)getCurrentTime ();
    }
#endif
  };

  //! @brief Stop the timer and return the elapsed time in ms
  //!
  //! @return The number of milliseconds since the timer was started,
  //!         0.0 if the timer is not currently running
  //!
  inline double stop ()
  {
    if (!running_)
    {
      return 0.0;
    }

#ifdef WIN32
    running_ = false;
    if (highRes_)
    {
      QueryPerformanceCounter (&hRsample_);
      timeDiff_ = hRsample_.QuadPart - hRstart_.QuadPart;
      return ((double)timeDiff_ * 1000.0 /
              (double)frequency_.QuadPart);
    }
    else
    {
      lRsample_ = (long)getCurrentTime ();
      return (lRsample_ - lRstart_);
    }
#endif    
  };

  //! @brief Sample the clock and report the length of time that has
  //!        passed since the timer was started/restarted.
  //!
  //! @return The number of milliseconds since the timer was started,
  //!         0.0 if the timer is not currently running
  //!
  inline double elapsedTime ()
  {
    if (!running_)
    {
      return 0.0;
    }

#ifdef WIN32
    if (highRes_)
    {
      QueryPerformanceCounter (&hRsample_);
      timeDiff_ = hRsample_.QuadPart - hRstart_.QuadPart;
      return ((double)timeDiff_ * 1000.0 /
              (double)frequency_.QuadPart);
    }
    else
    {
      lRsample_ = (long)getCurrentTime ();
      return (lRsample_ - lRstart_);
    }
#endif
  };

  //! @brief Wait until a specific amount of time has passed
  //!
  //! @param ms The time to wait in milliseconds
  //!
  inline void waitUntil (double ms)
  {
    if (highRes_)
    {
      restart ();
      while (elapsedTime () < ms)
      {
#ifdef WIN32
        Sleep (1);
#else
        usleep(1000);
#endif
      }
      stop ();
    }
    else
    {
#ifdef WIN32
      Sleep ((DWORD)ms);
#else
      usleep(ms * 1000);
#endif
    }
  };

private:
  //! @brief High resolution timer start time
  //!
  LARGE_INTEGER hRstart_;

  //! @brief High resolution timer sample time
  //!
  LARGE_INTEGER hRsample_;

  //! @brief High resolution system timer frequency
  //!
  LARGE_INTEGER frequency_;

  //! @brief Low resolution timer start time
  //!
  long lRstart_;

  //! @brief Low resolution timer sample time
  //!
  long lRsample_;

  //! @brief High resolution time sample difference variable
  //!
  LONGLONG timeDiff_;

  //! @brief Whether the timer is currently running
  //!
  bool running_;

  //! @brief Whether the system is capable of handling high
  //!        resolution clock sampling
  //!
  bool highRes_;
}; // class timer

#endif  // CRPI_H_
