///////////////////////////////////////////////////////////////////////////////
//
//  Original System: ISD CRPI
//  Subsystem:       Robot Interface
//  Workfile:        CrpiDemoHack.cpp
//  Revision:        1.0 - 08 April, 2014
//  Author:          J. Marvel
//
//  Description
//  ===========
//  A hack of an interface for the Robotiq gripper on the Kuka arm.
//
///////////////////////////////////////////////////////////////////////////////

#include "crpi_demo_hack.h"

using namespace std;

namespace crpi_robot
{
  LIBRARY_API CrpiDemoHack::CrpiDemoHack (CrpiRobotParams &params) :
    gripperCoupled(false),
    AGVCoupled(false)
  {
    arm_ = new robotArm (params);
    hand_ = new robotHand (params);
    strcpy (gripperName, "nothing");
    strcpy (objectName, "nothing");
  }


  LIBRARY_API CrpiDemoHack::~CrpiDemoHack ()
  {
    delete [] hand_;
    delete [] arm_;
  }

  LIBRARY_API CanonReturn CrpiDemoHack::ApplyCartesianForceTorque (robotPose robotForceTorque, vector<bool> activeAxes, vector<bool> manipulator)
  {

	return CANON_SUCCESS;
  }

  LIBRARY_API CanonReturn CrpiDemoHack::ApplyJointTorque (robotAxes robotJointTorque)
  {
	return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiDemoHack::SetTool (double percent)
  {
    int param;
    if (strcmp(gripperName, "gripper_parallel") == 0)
    {
      return arm_->SetTool(percent);
    }
    else
    {
      if (percent < 0.5)
      {
        param=100;
        SetParameter("POSITION_FINGER_A", &param);
        param=130;
        SetParameter("POSITION_FINGER_B", &param);
        SetParameter("POSITION_FINGER_C", &param);
      }
      return hand_->SetTool(percent);
    }
  }


  LIBRARY_API CanonReturn CrpiDemoHack::Couple (const char *targetID)
  {
    //! Robot arm and hand are permanently coupled in this hack.
    //! Coupling takes the effect of reconfiguring the robot hand.
    if (gripperCoupled)
    {
      //! Cannot couple a new gripper if one is already coupled to the robot; decouple first
      return CANON_REJECT;
    }

    if (strcmp(targetID, "gripper_gear") == 0)
    {
      hand_->Couple(targetID);
    }
    else if (strcmp(targetID, "gripper_top_cover") == 0)
    {
      hand_->Couple(targetID);
    }
    else if (strcmp(targetID, "gripper_bottom_cover") == 0)
    {
      hand_->Couple(targetID);
    }
    else if (strcmp(targetID, "gripper_finger_test_a") == 0)
    {
      hand_->Couple(targetID);
    }
    else if (strcmp(targetID, "gripper_finger_test_b") == 0)
    {
      hand_->Couple(targetID);
    }
      else if (strcmp(targetID, "gripper_finger_test_c") == 0)
    {
      hand_->Couple(targetID);
    }
    else if (strcmp(targetID, "gripper_parallel") == 0)
    {
      //! Fall through (arm-controlled gripper)
    }
    else
    {
      return CANON_FAILURE;
    }
    arm_->Couple(targetID);

    gripperCoupled = true;
    strcpy (gripperName, targetID);
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiDemoHack::Dwell (int *events, double *params, int numEvents)
  {
    return arm_->Dwell(events, params, numEvents);
  }


  LIBRARY_API CanonReturn CrpiDemoHack::Message (const char *message)
  {
    v1 = arm_->Message (message);
    v2 = hand_->Message (message);

    if (v1 == CANON_REJECT || v2 == CANON_REJECT)
    {
      return CANON_REJECT;
    }
    else if (v1 == CANON_FAILURE || v2 == CANON_FAILURE)
    {
      return CANON_FAILURE;
    }
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiDemoHack::MoveStraightTo (robotPose pose)
  {
    return arm_->MoveStraightTo (pose);
  }


  LIBRARY_API CanonReturn CrpiDemoHack::MoveThroughTo (robotPose *poses,
                                                   int numPoses,
                                                   robotPose *accelerations,
                                                   robotPose *speeds,
                                                   robotPose *tolerances)
  {
    return arm_->MoveThroughTo (poses, numPoses, accelerations, speeds, tolerances);
  }


  LIBRARY_API CanonReturn CrpiDemoHack::MoveTo (robotPose pose)
  {
    return arm_->MoveTo (pose);
  }


  LIBRARY_API CanonReturn CrpiDemoHack::Decouple (const char *targetID)
  {
    //! Robot arm and hand are permanently attached in this hack
    if (!gripperCoupled)
    {
      //! Cannot decouple if already decoupled
      return CANON_FAILURE;
    }

    if ((strcmp(targetID, gripperName) != 0) && (strcmp(targetID, objectName) != 0))
    {
      return CANON_FAILURE;
    }

    if (strcmp(targetID, gripperName) == 0)
    {
      strcpy (gripperName, "nothing");
      gripperCoupled = false;
    }
    else if (strcmp(targetID, objectName) == 0)
    {
      strcpy (objectName, "nothing");
    }
    
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiDemoHack::GetRobotAxes (robotAxes *axes)
  {
    return arm_->GetRobotAxes (axes);
  }


  LIBRARY_API CanonReturn CrpiDemoHack::GetRobotForces (robotPose *forces)
  {
    return arm_->GetRobotForces (forces);
  }
  

  LIBRARY_API CanonReturn CrpiDemoHack::GetRobotIO (robotIO *io)
  {
    return arm_->GetRobotIO (io);
  }


  LIBRARY_API CanonReturn CrpiDemoHack::GetRobotPose (robotPose *pose)
  {
    return arm_->GetRobotPose (pose);
  }
  

  LIBRARY_API CanonReturn CrpiDemoHack::GetRobotSpeed (robotPose *speed)
  {
    return arm_->GetRobotSpeed (speed);
  }


  LIBRARY_API CanonReturn CrpiDemoHack::GetRobotSpeed (robotAxes *speed)
  {
    return arm_->GetRobotSpeed (speed);
  }


  LIBRARY_API CanonReturn CrpiDemoHack::GetRobotTorques (robotAxes *torques)
  {
    return arm_->GetRobotTorques (torques);
  }


  LIBRARY_API CanonReturn CrpiDemoHack::MoveAttractor (robotPose pose)
  {
    return arm_->MoveAttractor (pose);
  }


  LIBRARY_API CanonReturn CrpiDemoHack::MoveToAxisTarget (robotAxes axes)
  {
    return arm_->MoveToAxisTarget (axes);
  }


  LIBRARY_API CanonReturn CrpiDemoHack::RunProgram (const char *programName, CRPIProgramParams params)
  {
    return arm_->RunProgram (programName, params);
  }


  LIBRARY_API CanonReturn CrpiDemoHack::SetAbsoluteAcceleration (double acceleration)
  {
    return arm_->SetAbsoluteAcceleration (acceleration);
  }


  LIBRARY_API CanonReturn CrpiDemoHack::SetAbsoluteSpeed (double speed)
  {
    return arm_->SetAbsoluteSpeed (speed);
  }


  LIBRARY_API CanonReturn CrpiDemoHack::SetAngleUnits (const char *unitName)
  {
    return arm_->SetAngleUnits (unitName);
  }


  LIBRARY_API CanonReturn CrpiDemoHack::SetAxialSpeeds (double *speeds)
  {
    return arm_->SetAxialSpeeds (speeds);
  }


  LIBRARY_API CanonReturn CrpiDemoHack::SetAxialUnits (const char **unitNames)
  {
    return arm_->SetAxialUnits (unitNames);
  }


  LIBRARY_API CanonReturn CrpiDemoHack::SetEndPoseTolerance (robotPose tolerance)
  {
    return arm_->SetEndPoseTolerance (tolerance);
  }


  LIBRARY_API CanonReturn CrpiDemoHack::SetIntermediatePoseTolerance (robotPose *tolerances)
  {
    return arm_->SetIntermediatePoseTolerance (tolerances);
  }


  LIBRARY_API CanonReturn CrpiDemoHack::SetLengthUnits (const char *unitName)
  {
    return arm_->SetLengthUnits (unitName);
  }


  LIBRARY_API CanonReturn CrpiDemoHack::SetParameter (const char *paramName, void *paramVal)
  {
    //! Unfortunately we have to assume we're addressing one robot or the other.  Here, we
    //! assume we're addressing the hand.  Hacking together two robots like this is going
    //! to result in reduced functionality.
    return hand_->SetParameter (paramName, paramVal);
  }

  LIBRARY_API CanonReturn CrpiDemoHack::SetRelativeAcceleration (double percent)
  {
    return arm_->SetRelativeAcceleration (percent);
  }


  LIBRARY_API CanonReturn CrpiDemoHack::SetRelativeSpeed (double percent)
  {
    return arm_->SetRelativeSpeed (percent);
  }


  LIBRARY_API CanonReturn CrpiDemoHack::SetRobotIO (robotIO io)
  {
    //! TODO
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiDemoHack::SetRobotDO (int dig_out, bool val)
  {
    //! TODO
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiDemoHack::StopMotion (int condition)
  {
    v1 = arm_->StopMotion (condition);
    v2 = hand_->StopMotion (condition);

    if (v1 == CANON_REJECT || v2 == CANON_REJECT)
    {
      return CANON_REJECT;
    }
    else if (v1 == CANON_FAILURE || v2 == CANON_FAILURE)
    {
      return CANON_FAILURE;
    }
    return CANON_SUCCESS;
  }

} // Robot
