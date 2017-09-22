///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Motion Primitives
//  Workfile:        MatHandlingPrims.cpp
//  Revision:        15 June, 2017
//  Author:          J. Marvel
//
//  Description
//  ===========
// 
///////////////////////////////////////////////////////////////////////////////

#include "MatHandlingPrims.h"

#include <iostream>

namespace MotionPrims
{
  LIBRARY_API MatHandling::MatHandling ()
  {
  }


  LIBRARY_API MatHandling::~MatHandling()
  {
  }
  
  /*
  LIBRARY_API bool MatHandling::motionCfg (assemblyParams &aP)
  {
    return true;
  } //motionCfg
  */

  LIBRARY_API CanonReturn MatHandling  ::ClearMotions ()
  {
    //! Not yet implemented
    return CANON_REJECT;
  } //ClearSearch


  LIBRARY_API CanonReturn MatHandling::RunMatHandleStep (int counter, robotPose &robPose, robotPose &newPose, robotIO &ios)
  {
    //! Not yet imnplemented
    return CANON_REJECT;
/*
    bool state;
    CanonReturn returnMe = CANON_RUNNING;
    vector<terminatorParams>::iterator tpi;
    vector<assemblyParams>::iterator api;
    int index;

    //! known_freq = get_freq(10, new_search)

    state = true;
    curPose_ = robPose;
    curIO_ = ios;

    if (termParams_.size() < 1)
    {
      //! At least one termination condition must be defined
      return CANON_FAILURE;
    }

    if (newSearch)
    {
      initPose_ = robPose;

      for (tpi = termParams_.begin(); tpi != termParams_.end(); ++tpi)
      {
        if (tpi->tType == TERMINATOR_TIMER)
        {
          timer_.stopTimer();
          timer_.startTimer();
        }        
      } // for (tpi ...)
      newSearch = false;
    } // if (newSearch)

    newPose_ = initPose_;

    for (api = assemblyParams_.begin(); api != assemblyParams_.end(); ++api)
    {
      state = state && applyOffset(counter, *api);
    }

    if (state)
    {
      state &= testTerm (index);
    }
    else
    {
      newSearch = true;
      return CANON_FAILURE;
    }

    if (state)
    {
      newPose = newPose_;
      //! Termination condition not met, still searching
//      state &= (robot_->MoveTo(newPose_) == CANON_SUCCESS);
      //counter += 1;
    }
    else
    {
      //! Search terminated (success, error, fail)
      //! Type of terminator condition found at termParms_.at(index).tType
      returnMe = termParams_.at(index).rType;
      newSearch = true;
    }
    
    return returnMe;
    */
  } //RunAssembly

} // namespace MotionPrims