///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Motion Primitives
//  Workfile:        AssemblyPrims.h
//  Revision:        15 December, 2014
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Motion primitives for assembly search functions.
///////////////////////////////////////////////////////////////////////////////

#ifndef ASSEMBLY_PRIMS
#define ASSEMBLY_PRIMS

#include "crpi.h"
#include <vector>
#include "crpi_robot.h"

namespace MotionPrims
{
  //! @brief Assembly search type identifiers
  typedef enum
  {
    ASSEMBLY_RANDOM = 0,
    ASSEMBLY_SOBOL,
    ASSEMBLY_SPIRAL,
    ASSEMBLY_SPIRAL_OPTIMAL,
    ASSEMBLY_SQ_SPIRAL,
    ASSEMBLY_RASTER,
    ASSEMBLY_TILT,
    ASSEMBLY_ROTATION,
    ASSEMBLY_CIRCLE,
    ASSEMBLY_HOP,
    ASSEMBLY_LINEAR,
    ASSEMBLY_CONST_OFFSET
  } SearchType;

  //! @brief Termination condition identifiers
  //!
  typedef enum
  {
    TERMINATOR_EXTSIGNAL = 0,
    TERMINATOR_CONTACT,
    TERMINATOR_TIMER,
    TERMINATOR_DISTANCE,
    TERMINATOR_REPETITION,
    TERMINATOR_NONE
  } TermType;

  //! @brief Basic timer functionality structure
  //!
  struct AssemblyTimer
  {
    //! @brief The time (in seconds) at which the timer was started
    //!
    double inittime;

    //! @brief Whether the timer has been started
    //!
    bool started;

    //! @brief Default constructor
    //!
    AssemblyTimer ()
    {
      inittime = -1.0f;
      started = false;
    }

    //! @brief Default destructor
    //!
    ~AssemblyTimer ()
    {
      started = false;
    }

    //! @brief Start the timer functionality by recording the time
    //!
    //! @return True if the timer was started, false if the timer is already running
    //!
    bool startTimer()
    {
      if (started)
      {
        return false;
      }
      started = true;
      inittime = ulapi_time();
      return true;
    }

    //! @brief Stop the timer functionality
    //!
    //! @return True if the timer was stopped, false if the timer was not running
    //!
    bool stopTimer()
    {
      if (!started)
      {
        return false;
      }
      started = false;
      return true;
    }

    //! @brief Get the amount of time (in seconds) since the timer was started
    //!
    //! @return The elapsed time if the timer is started, -1 otherwise
    //!
    double timeElapsed()
    {
      if (started)
      {
        return ulapi_time() - inittime;
      }
      else
      {
        return -1.0f;
      }
    }

  };


  //! @brief A collection of various parameters that define the assembly search primitives
  //!
  struct LIBRARY_API assemblyParams
  {
    SearchType sType;
    bool randWalk;
    double speed;
    double length;
    double width;
    int turns;
    double x;
    double y;
    double z;
    double magnitude;
    double radius;
    double radiusOffset;
    double totalLength;
    double lengthDelta;
    double lengthStep;
    double xOffset;
    double yOffset;
    double zOffset;
    double degOffset;
    double degOffsetDelta;
    double thetaMax;
    double rasterRatio;
    double pitch;
    int totalPoints;
  };

  //! @brief A collection of termination conditions that define when to stop the assembly search
  //!
  struct LIBRARY_API terminatorParams
  {
    TermType tType;
    CanonReturn rType;
    bool result;
    double threshold;
    int timer;
    double endTime;
    int signal;
    double xDelta;
    double yDelta;
    double zDelta;
  };

  //! @ingroup MotionPrims
  //!
  class LIBRARY_API Assembly
  {
  public:
    //! @brief Default constructor
    //!
    Assembly ();

    //! @brief Default destructor
    //!
    ~Assembly ();

    //! @brief Searches the defined region using a random walk
    //!
    //! @param radius Radius of the search region
    //! @param walk   Allow the search to explore beyond the radius (true)
    //!
    //! @return CANON_SUCCESS if the search routine was added successfully, CANON_FAILURE otherwise
    //!
    CanonReturn AddSearchRandom (double radius, bool walk = false);
  
    //! @brief Searches the region using the quasi-random sobol sequence 
    //!
    //! @param radius Radius of the search region
    //! @param walk   Allow the search to explore beyond the radius (false)
    //!
    //! @return CANON_SUCCESS if the search routine was added successfully, CANON_FAILURE otherwise
    //!
    CanonReturn AddSearchSobol(double radius, bool walk = false);

    //! @brief Add a spiral search (moving the TCP on the XY plane in a spiral pattern) to the queue
    //!
    //! @param turns  The number of turns in the spiral
    //! @param radius The radius of the search region in mm
    //! @param speed  The speed of the search in degrees per second
    //!
    //! @return CANON_SUCCESS if the search routine was added successfully, CANON_FAILURE otherwise
    //!
    CanonReturn AddSearchSpiral (int turns, double radius, double speed);

    //! @brief Add a spiral search, optimized based on the expected hole clearance
    //!
    //! @param clearance Clearance between peg and hole in mm, which is used to determine distance between turnings
    //! @param radius    Radius of the search region in mm
    //! @param speed     The speed of the search in degrees per second
    //!
    //! @return CANON_SUCCESS if the search routine was added successfully, CANON_FAILURE otherwise
    //!
    CanonReturn AddSearchSpiralOptimal(double clearance, double radius, double speed);

    //! @brief Add a square spiral search routine
    //!
    //! @param step   Search step size in mm.
    //! @param radius Search radius in mm
    //!
    //! @return CANON_SUCCESS if the search routine was added successfully, CANON_FAILURE otherwise
    //!
    CanonReturn AddSearchSqSpiral(double step, double radius);

    //! @brief Add a raster search (moving the part along the XY plane in a raster pattern) to the queue
    //!
    //! @param rasters The number of rasters to create in the search area
    //! @param width   The width of the search area in mm
    //! @param length  The length of the search area in mm
    //! @param speed   The speed (mm/s) for searching along the raster pattern
    //!
    //! @return CANON_SUCCESS if the search routine was added successfully, CANON_FAILURE otherwise
    //!
    CanonReturn AddSearchRaster (int rasters, double width, double length, double speed);

    //! @brief Add a tilt search to the queue
    //!
    //! @param TODO
    //!
    //! @return CANON_SUCCESS if the search routine was added successfully, CANON_FAILURE otherwise
    //!
    CanonReturn AddSearchTilt ();

    //! @brief Add a rotational search (rotating the part back and forth along the Z axis) to the queue
    //!
    //! @param range The maximum rotational offset
    //! @param speed The speed at which the part is rotated
    //!
    //! @return CANON_SUCCESS if the search routine was added successfully, CANON_FAILURE otherwise
    //!
    CanonReturn AddSearchRotation (double range, double speed);

    //! @brief Add a circular search (moving the TCP in a circle) to the queue
    //!
    //! @param radius The radius of the circle pattern
    //! @param speed  The speed at which the TCP is moved along the circle
    //!
    //! @return CANON_SUCCESS if the search routine was added successfully, CANON_FAILURE otherwise
    //!
    CanonReturn AddSearchCircle (double radius, double speed);

    //! @brief Add a hopping search to the queue
    //!
    //! @param magnitude The maximum displacement from the current position
    //! @param frequency The rate at which the TCP is moved up and down
    //!
    //! @return CANON_SUCCESS if the search routine was added successfully, CANON_FAILURE otherwise
    //!
    CanonReturn AddSearchHop (double magnitude, double frequency);

    //! @brief Add a linear search to the queue
    //!
    //! @param xoff  TODO
    //! @param yoff  TODO
    //! @param zoff  TODO
    //! @param speed TODO
    //!
    //! @return CANON_SUCCESS if the search routine was added successfully, CANON_FAILURE otherwise
    //!
    CanonReturn AddSearchLinear (double xoff, double yoff, double zoff, double speed);

    //! @brief Add a constant distance terminator condition
    //!
    //! @param xoff The X axis distance threshold
    //! @param yoff The Y axis distance threshold
    //! @param zoff The Z axis distance threshold
    //!
    //! @return CANON_SUCCESS if the search routine was added successfully, CANON_FAILURE otherwise
    //!
    CanonReturn AddSearchConstOffset (double xoff, double yoff, double zoff);

    //! @brief Add a time-out terminator condition
    //!
    //! @param rType   Specified return time upon meeting the termination condition
    //! @param timeout The value of the timer in seconds
    //!
    //! @return CANON_SUCCESS if the termination condition was added successfully, CANON_FAILURE otherwise
    //!
    CanonReturn AddTerminatorTimer (CanonReturn rType, double timeout);

    //! @brief Add a condition that terminates the search upon contact with an object/surface
    //!
    //! @param rType     Specified return time upon meeting the termination condition
    //! @param threshold Force threshold for determining contact has been made
    //!
    //! @return CANON_SUCCESS if the termination condition was added successfully, CANON_FAILURE otherwise
    //!
    CanonReturn AddTerminatorContact (CanonReturn rType, double threshold);

    //! @brief TODO
    //!
    //! @param rType  Specified return time upon meeting the termination condition
    //! @param signal The DI/O input number to monitor.  When the prescribed input goes high, the search must stop
    //!
    //! @return CANON_SUCCESS if the termination condition was added successfully, CANON_FAILURE otherwise
    //!
    CanonReturn AddTerminatorSignal (CanonReturn rType, int signal);

    //! @brief TODO
    //!
    //! @param rType Specified return time upon meeting the termination condition
    //! @param x     TODO
    //! @param y     TODO
    //! @param z     TODO
    //! @param total TODO
    //!
    //! @return CANON_SUCCESS if the termination condition was added successfully, CANON_FAILURE otherwise
    //!
    CanonReturn AddTerminatorDistance (CanonReturn rType, double x, double y, double z, double total);

    //! @brief TODO
    //!
    //! @param rType Specified return time upon meeting the termination condition
    //! @param reps  TODO
    //!
    //! @return CANON_SUCCESS if the termination condition was added successfully, CANON_FAILURE otherwise
    //!
    CanonReturn AddTerminatorRepetition (CanonReturn rType, int reps);

    //! @brief Clear the search and terminator parameters
    //!
    //! @return CANON_SUCCESS if paramters successfully cleared, CANON_FAILURE otherwise
    //!
    CanonReturn ClearSearch ();

    //! @brief TODO
    //!
    //! @param counter The current counter step in the process
    //! @param robPose The current pose of the robot
    //! @param newPose THe next target pose of the robot, populated by this function
    //! @param ios     TODO
    //!
    //! @return TODO
    //!
    CanonReturn RunAssemblyStep (int counter, robotPose &robPose, robotPose &newPose, robotIO &ios);

  private:

    //! @brief Whether or not the current assembly search is the first instance it has been configured
    //!
    bool newSearch;

    //! @brief Robot pose at the start of the search routine
    //!
    robotPose initPose_;

    //! @brief Target robot position in 6DOF space
    //!
    robotPose newPose_;

    //! @brief Current robot position in 6DOF space
    //!
    robotPose curPose_;

    //! @brief Current robot IO
    //!
    robotIO curIO_;

    //! @brief Timer for maintaining correct motion profile and tracking timer-based termination conditions
    //!
    AssemblyTimer timer_;

    //! @brief Collection of termination condition parameters
    //!
    vector<terminatorParams> termParams_;

    //! @brief Collection of assembly search routine parameters
    //!
    vector<assemblyParams> assemblyParams_;

    //! @brief Update frequency
    //!
    double curFreq_;

    //! @brief TODO
    //!
    int *sqs_x;
    int *sqs_y;

    //! @brief TODO
    //!
    //! @param aP TODO
    //!
    //! @return TODO
    //!
    CanonReturn AddSearch (assemblyParams &aP);

    //! @brief TODO
    //!
    //! @param tP TODO
    //!
    //! @return TODO
    //!
    CanonReturn AddTerminator (terminatorParams &tP);

    //! @brief TODO
    //! 
    //! @param aP TODO
    //!
    //! @return TODO
    //!
    bool motionCfg (assemblyParams &aP);

    //! @brief TODO
    //!
    //! @param counter
    //! @param ap
    //!
    //! @return TODO
    //!
    bool applyOffset (int counter, assemblyParams &ap);

    //! @brief TODO
    //!
    //! @return TODO
    //!
    bool testTerm (int &index);
    
    //! @brief TODO
    //!
    //! @param table_size TODO
    //!
    void init_sqs_table(int table_size);
  }; // Assembly

} // namespace MotionPrims

#endif