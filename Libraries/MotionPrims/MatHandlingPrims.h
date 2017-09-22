///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Motion Primitives
//  Workfile:        MatHandlingPrims.h
//  Revision:        15 June, 2017
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Motion primitives for material handling functions.
///////////////////////////////////////////////////////////////////////////////

#ifndef MATERIAL_HANDLING_PRIMS
#define MATERIAL_HANDLING_PRIMS

#include "crpi.h"
#include <vector>
#include "crpi_robot.h"

namespace MotionPrims
{
  //! @ingroup MotionPrims
  //!
  class LIBRARY_API MatHandling
  {
  public:
    //! @brief Default constructor
    //!
    MatHandling ();

    //! @brief Default destructor
    //!
    ~MatHandling ();

    //! @brief TODO
    //!
    //! @param hover
    //! @param acquire
    //! @param retract
    //!
    //! @return CANON_SUCCESS if the search routine was added successfully, CANON_FAILURE otherwise
    //!
    CanonReturn AddGetPart(robotPose &hover, robotPose &acquire, robotPose &retract);
    CanonReturn AddGetPart(robotPose &acquire, robotPose &relHover);



    CanonReturn AddMovePart(robotPose &target);

    CanonReturn AddPutPart(robotPose &hover, robotPose &place, robotPose &retract);

    //! @brief Add a spiral search (moving the TCP on the XY plane in a spiral pattern) to the queue
    //!
    //! @param turns  The number of turns in the spiral
    //! @param radius The radius of the search region in mm
    //! @param speed  The speed of the search in degrees per second
    //!
    //! @return CANON_SUCCESS if the search routine was added successfully, CANON_FAILURE otherwise
    //!
    CanonReturn AddSearchSpiral (int turns, double radius, double speed);

    //! @brief Clear the search and terminator parameters
    //!
    //! @return CANON_SUCCESS if paramters successfully cleared, CANON_FAILURE otherwise
    //!
    CanonReturn ClearMotions ();

    //! @brief TODO
    //!
    //! @param counter TODO
    //!
    //! @return TODO
    //!
    CanonReturn RunMatHandleStep (int counter, robotPose &robPose, robotPose &newPose, robotIO &ios);

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

  }; // MatHandling

} // namespace MotionPrims

#endif