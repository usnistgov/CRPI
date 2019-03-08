///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Robot Interface
//  Workfile:        crpi_robot.cpp
//  Revision:        1.0 - 11 March, 2014
//                   2.0 - 20 February, 2015 - Transition from CRCL to CRPI.
//                                             Added CRCL XML handler.
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Robot interface definitions.
//
///////////////////////////////////////////////////////////////////////////////

#include "crpi_robot.h"

#include <fstream>
#include <iostream>
using namespace std;

#include "crpi_robotiq.h"
#include "crpi_kuka_lwr.h"
#include "crpi_schunk_sdh.h"
#include "crpi_universal.h"
#include "crpi_robot_xml.h"
#include "crpi_allegro.h"
#include "crpi_abb.h"

#include "crpi_crcl_robot.h"	// FMP

//#define NOISY

//! Explicit instantiations
template class LIBRARY_API crpi_robot::CrpiRobot<crpi_robot::CrpiSchunkSDH>;
template class LIBRARY_API crpi_robot::CrpiRobot<crpi_robot::CrpiRobotiq>;
template class LIBRARY_API crpi_robot::CrpiRobot<crpi_robot::CrpiKukaLWR>;
template class LIBRARY_API crpi_robot::CrpiRobot<crpi_robot::CrpiUniversal>;
template class LIBRARY_API crpi_robot::CrpiRobot<crpi_robot::CrpiAllegro>;
template class LIBRARY_API crpi_robot::CrpiRobot<crpi_robot::CrpiAbb>;

template class LIBRARY_API crpi_robot::CrpiRobot<crpi_robot::CrpiCrcl>; // FMP

namespace crpi_robot
{
  template <class T> LIBRARY_API CrpiRobot<T>::CrpiRobot (const char *initPath, bool bypass)
  {
    robotparams_ = new CrpiRobotParams();
    bypass_ = bypass;

    strcpy_s(robotparams_->initPath, initPath); // FMP
    robInterface_ = NULL;	// FMP

    char line[1024];
    ifstream inputs(initPath);
    if (!inputs)
    {
      cout << "Could not open file " << initPath << ". Robot not initialized." << endl;
      return;
    }
    CrpiRobotXml robXML(robotparams_);
    stringstream grabbyGrabby;
    grabbyGrabby.str(string());

    while (inputs.getline(line, 1024))
    {
      grabbyGrabby << line;
    }
    inputs.close();

#ifdef NOISY
    cout << grabbyGrabby.str().c_str() << endl;
#endif
    robXML.parse(grabbyGrabby.str().c_str());

    inputs.close();

    bool state;
    if (!robotparams_->usedMatrix)
    {
      cout << "no matrix used" << endl;
      //! Update to matrix representation
      Math::pose ptemp = robotparams_->toWorld->pose();
      state = robotparams_->toWorldMatrix->RPYMatrixConvert(ptemp, true);
      char lineout[4096];
      ofstream out(initPath);
      robXML.encode(lineout);
      out << lineout;
    }

    robInterface_ = (bypass_ ? NULL : new T(*robotparams_));
    crpiparams_ = new CrpiXmlParams();
    crclxml_ = new CrclXml(crpiparams_);
    crpixml_ = new CrpiXml(crpiparams_);
    rotMatrix_ = new matrix(3, 3);
    crpiparams_->toolName = "Nothing";
    crpiparams_->toolVal = 0.0f;
    v1_ = new vector3D;
    v2_ = new vector3D;
    v3_ = new vector3D;
    crpiparams_->status = CANON_REJECT;
  }

  template <class T> LIBRARY_API CrpiRobot<T>::~CrpiRobot ()
  {
    if (!bypass_)
    {
      delete robInterface_;
    }
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::SetTool (double percent)
  {
    if (bypass_)
    {
      return CANON_SUCCESS;
    }

    CanonReturn val;
    crpiparams_->status = CANON_RUNNING;
    crpiparams_->toolVal = percent;
    val = robInterface_->SetTool (percent);
    crpiparams_->status = val;
    return val;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::ApplyCartesianForceTorque (robotPose &robotForceTorque, vector<bool> activeAxes, vector<bool> manipulator)
  {
    if (bypass_)
    {
      return CANON_SUCCESS;
    }

    CanonReturn val;
    crpiparams_->status = CANON_RUNNING;
    val = robInterface_->ApplyCartesianForceTorque (robotForceTorque, activeAxes, manipulator);
    crpiparams_->status = val;
    return val;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::ApplyJointTorque (robotAxes &robotJointTorque)
  {
    CanonReturn val;

    if (bypass_) {
      return CANON_SUCCESS;
    }

    crpiparams_->status = CANON_RUNNING;
    val = robInterface_->ApplyJointTorque(robotJointTorque);
    crpiparams_->status = val;
    return val;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::Couple (const char *targetID)
  {
    if (bypass_)
    {
      return CANON_SUCCESS;
    }

    CanonReturn val;
    crpiparams_->status = CANON_RUNNING;
    crpiparams_->toolName = targetID;
    val = robInterface_->Couple(targetID);
    crpiparams_->status = val;
    return val;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::GetRobotAxes (robotAxes *axes)
  {
    if (bypass_)
    {
      robotAxes temp;
      *axes = temp;
      *crpiparams_->axes = temp;
      return CANON_SUCCESS;
    }

    CanonReturn val;
    crpiparams_->status = CANON_RUNNING;
    val = robInterface_->GetRobotAxes (axes);
    *crpiparams_->axes = *axes;
    crpiparams_->status = val;
    return val;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::GetRobotForces (robotPose *forces)
  {
    if (bypass_)
    {
      robotPose temp;
      *forces = temp;
      *crpiparams_->forces = temp;
      return CANON_SUCCESS;
    }

    CanonReturn val;
    crpiparams_->status = CANON_RUNNING;
    val = robInterface_->GetRobotForces (forces);
    *crpiparams_->forces = *forces;
    return val;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::GetRobotIO (robotIO *io)
  {
    if (bypass_)
    {
      robotIO temp;
      *io = temp;
      *crpiparams_->io = temp;
      return CANON_SUCCESS;
    }

    CanonReturn val;
    crpiparams_->status = CANON_RUNNING;
    val = robInterface_->GetRobotIO (io);
    *crpiparams_->io = *io;
    crpiparams_->status = val;
    return val;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::GetRobotPose (robotPose *pose)
  {
    if (bypass_)
    {
      robotPose temp;
      *pose = temp;
      *crpiparams_->pose = temp;
      return CANON_SUCCESS;
    }

    CanonReturn val;
    crpiparams_->status = CANON_RUNNING;
    //cout << "robot: get pose" << endl;
    val = robInterface_->GetRobotPose (pose);
    //cout << "robot: do math" << endl;
    Math::pose ptemp = pose->pose();
    rotMatrix_->RPYMatrixConvert (ptemp, (angleUnits_ == DEGREE));
    crpiparams_->xaxis.i = rotMatrix_->at (0, 0);
    crpiparams_->xaxis.j = rotMatrix_->at (1, 0);
    crpiparams_->xaxis.k = rotMatrix_->at (2, 0);

    crpiparams_->zaxis.i = rotMatrix_->at (0, 2);
    crpiparams_->zaxis.j = rotMatrix_->at (1, 2);
    crpiparams_->zaxis.k = rotMatrix_->at (2, 2);
    crpiparams_->status = val;
    *crpiparams_->pose = ptemp;
    return val;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::GetRobotSpeed (robotAxes *speed)
  {
    if (bypass_)
    {
      robotAxes temp;
      *speed = temp;
      return CANON_SUCCESS;
    }

    CanonReturn val;
    crpiparams_->status = CANON_RUNNING;
    val = robInterface_->GetRobotSpeed (speed);
    crpiparams_->status = val;
    return val;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::GetRobotSpeed (robotPose *speed)
  {
    if (bypass_)
    {
      robotPose temp;
      *speed = temp;
      return CANON_SUCCESS;
    }
    CanonReturn val;
    crpiparams_->status = CANON_RUNNING;
    val = robInterface_->GetRobotSpeed (speed);
    crpiparams_->status = val;
    return val;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::GetRobotTorques (robotAxes *torques)
  {
    if (bypass_)
    {
      robotAxes temp;
      *torques = temp;
      *crpiparams_->torques = temp;
      return CANON_SUCCESS;
    }
    CanonReturn val;
    crpiparams_->status = CANON_RUNNING;
    val = robInterface_->GetRobotTorques (torques);
    *crpiparams_->torques = *torques;
    crpiparams_->status = val;
    return val;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::Message (const char *message)
  {
    if (bypass_)
    {
      return CANON_SUCCESS;
    }
    CanonReturn val;
    crpiparams_->status = CANON_RUNNING;
    val = robInterface_->Message (message);
    crpiparams_->status = val;
    return val;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::MoveStraightTo (robotPose &pose)
  {
    if (bypass_)
    {
      return CANON_SUCCESS;
    }
    CanonReturn val;
    crpiparams_->status = CANON_RUNNING;
    val = robInterface_->MoveStraightTo (pose);
    crpiparams_->status = val;
    return val;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::MoveThroughTo (robotPose *poses,
                                                                           int numPoses,
                                                                           robotPose *accelerations,
                                                                           robotPose *speeds,
                                                                           robotPose *tolerances)
  {
    if (bypass_)
    {
      return CANON_SUCCESS;
    }
    CanonReturn val;
    crpiparams_->status = CANON_RUNNING;
    val = robInterface_->MoveThroughTo (poses, numPoses, accelerations, speeds, tolerances);
    crpiparams_->status = val;
    return val;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::MoveTo (robotPose &pose)
  {
    if (bypass_)
    {
      return CANON_SUCCESS;
    }
    CanonReturn val;
    crpiparams_->status = CANON_RUNNING;
    val = robInterface_->MoveTo (pose);
    crpiparams_->status = val;
    return val;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::MoveAttractor (robotPose &pose)
  {
    if (bypass_)
    {
      return CANON_SUCCESS;
    }
    CanonReturn val;
    crpiparams_->status = CANON_RUNNING;
    val = robInterface_->MoveAttractor (pose);
    crpiparams_->status = val;
    return val;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::MoveToAxisTarget (robotAxes &axes)
  {
    if (bypass_)
    {
      return CANON_SUCCESS;
    }
    CanonReturn val;
    crpiparams_->status = CANON_RUNNING;
    val = robInterface_->MoveToAxisTarget (axes);
    crpiparams_->status = val;
    return val;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::SetAbsoluteAcceleration (double tolerance)
  {
    if (bypass_)
    {
      return CANON_SUCCESS;
    }
    CanonReturn val;
    crpiparams_->status = CANON_RUNNING;
    val = robInterface_->SetAbsoluteAcceleration (tolerance);
    crpiparams_->status = val;
    return val;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::SetAbsoluteSpeed (double speed)
  {
    if (bypass_)
    {
      return CANON_SUCCESS;
    }
    CanonReturn val;
    crpiparams_->status = CANON_RUNNING;
    val = robInterface_->SetAbsoluteSpeed (speed);
    crpiparams_->status = val;
    return val;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::SetAngleUnits (const char *unitName)
  {
    if (strcmp(unitName, "degree") == 0)
    {
      angleUnits_ = DEGREE;
    }
    else if (strcmp(unitName, "radian") == 0)
    {
      angleUnits_ = RADIAN;
    }
    else
    {
      return CANON_FAILURE;
    }

    if (bypass_)
    {
      return CANON_SUCCESS;
    }
    CanonReturn val;
    crpiparams_->status = CANON_RUNNING;
    val = robInterface_->SetAngleUnits (unitName);
    crpiparams_->status = val;

    return val;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::SetAxialSpeeds (double *speeds)
  {
    if (bypass_)
    {
      return CANON_SUCCESS;
    }
    CanonReturn val;
    crpiparams_->status = CANON_RUNNING;
    val = robInterface_->SetAxialSpeeds (speeds);
    crpiparams_->status = val;
    return val;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::SetAxialUnits (const char **unitNames)
  {
    if (bypass_)
    {
      return CANON_SUCCESS;
    }
    CanonReturn val;
    crpiparams_->status = CANON_RUNNING;
    val = robInterface_->SetAxialUnits (unitNames);
    crpiparams_->status = val;
    return val;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::SetEndPoseTolerance (robotPose &tolerance)
  {
    if (bypass_)
    {
      return CANON_SUCCESS;
    }
    CanonReturn val;
    crpiparams_->status = CANON_RUNNING;
    val = robInterface_->SetEndPoseTolerance (tolerance);
    crpiparams_->status = val;
    return val;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::SetIntermediatePoseTolerance (robotPose *tolerances)
  {
    if (bypass_)
    {
      return CANON_SUCCESS;
    }
    CanonReturn val;
    crpiparams_->status = CANON_RUNNING;
    val = robInterface_->SetIntermediatePoseTolerance (tolerances);
    crpiparams_->status = val;
    return val;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::SetLengthUnits (const char *unitName)
  {
    if (bypass_)
    {
      return CANON_SUCCESS;
    }
    CanonReturn val;
    crpiparams_->status = CANON_RUNNING;
    val = robInterface_->SetLengthUnits (unitName);
    crpiparams_->status = val;

    if (strcmp(unitName, "meter") == 0)
    {
      lengthUnits_ = METER;
    }
    else if (strcmp(unitName, "mm") == 0)
    {
      lengthUnits_ = MM;
    }
    else if (strcmp(unitName, "inch") == 0)
    {
      lengthUnits_ = INCH;
    }
    else
    {
      CANON_FAILURE;
    }
    return val;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::SetParameter (const char *paramName, void *paramVal)
  {
    if (bypass_)
    {
      return CANON_SUCCESS;
    }
    CanonReturn val;
    crpiparams_->status = CANON_RUNNING;
    val = robInterface_->SetParameter (paramName, paramVal);
    crpiparams_->status = val;
    return val;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::SetRelativeAcceleration (double percent)
  {
    if (bypass_)
    {
      return CANON_SUCCESS;
    }
    CanonReturn val;
    crpiparams_->status = CANON_RUNNING;
    val = robInterface_->SetRelativeAcceleration (percent);
    crpiparams_->status = val;
    return val;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::SetRelativeSpeed (double percent)
  {
    if (bypass_)
    {
      return CANON_SUCCESS;
    }
    CanonReturn val;
    crpiparams_->status = CANON_RUNNING;
    val = robInterface_->SetRelativeSpeed (percent);
    crpiparams_->status = val;
    return val;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::SetRobotIO (robotIO &io)
  {
    if (bypass_)
    {
      return CANON_SUCCESS;
    }
    CanonReturn val;
    crpiparams_->status = CANON_RUNNING;
    val = robInterface_->SetRobotIO (io);
    crpiparams_->status = val;
    return val;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::SetRobotDO (int dig_out, bool val)
  {
    if (bypass_)
    {
      return CANON_SUCCESS;
    }
    CanonReturn retval;
    crpiparams_->status = CANON_RUNNING;
    retval = robInterface_->SetRobotDO (dig_out, val);
    crpiparams_->status = retval;
    return retval;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::StopMotion (int condition)
  {
    if (bypass_)
    {
      return CANON_SUCCESS;
    }
    CanonReturn val;
    crpiparams_->status = CANON_RUNNING;
    val = robInterface_->StopMotion (condition);
    crpiparams_->status = val;
    return val;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::CrclXmlHandler (std::string& str)
  {
    crclxml_->parse(str); //! Populate the params_ structure based on the XML string

    //! Get the CRPI 6DOF pose from the 2-vector representation CRCL uses
    v1_->i = crpiparams_->xaxis.i;
    v1_->j = crpiparams_->xaxis.j;
    v1_->k = crpiparams_->xaxis.k;

    v2_->i = crpiparams_->zaxis.i;
    v2_->j = crpiparams_->zaxis.j;
    v2_->k = crpiparams_->zaxis.k;

    v3_->i = -((v1_->j * v2_->k) - (v1_->k * v2_->j));
    v3_->j = -((v1_->k * v2_->i) - (v1_->i * v2_->k));
    v3_->k = -((v1_->i * v2_->j) - (v1_->j * v2_->i));

    rotMatrix_->at(0, 0) = v1_->i;
    rotMatrix_->at(1, 0) = v1_->j;
    rotMatrix_->at(2, 0) = v1_->k;
    rotMatrix_->at(0, 1) = v3_->i;
    rotMatrix_->at(1, 1) = v3_->j;
    rotMatrix_->at(2, 1) = v3_->k;
    rotMatrix_->at(0, 2) = v2_->i;
    rotMatrix_->at(1, 2) = v2_->j;
    rotMatrix_->at(2, 2) = v2_->k;

    Math::pose ptemp;
    rotMatrix_->matrixRPYConvert (ptemp, (angleUnits_ == DEGREE));
    *crpiparams_->pose = ptemp;

    switch (crpiparams_->cmd)
    {
    case CmdCouple:
      return Couple (crpiparams_->str.c_str());
      break;
    case CmdDwell:
      //! No equivalent in CRPI
      return CANON_REJECT;
      break;
    case CmdEndCanon:
      //! No equivalence in CRPI
      return CANON_REJECT;
      break;
    case CmdGetRobotAxes:
      return GetRobotAxes (crpiparams_->axes);
      break;
    case CmdGetRobotIO:
      return GetRobotIO(crpiparams_->io);
      break;
    case CmdGetRobotPose:
      return GetRobotPose (crpiparams_->pose);
      break;
    case CmdInitCanon:
      //! No equivalence in CRPI
      return CANON_REJECT;
      break;
    case CmdMessage:
      return Message (crpiparams_->str.c_str());
      break;
    case CmdMoveAttractor:
      return MoveAttractor (*crpiparams_->pose);
      break;
    case CmdMoveStraightTo:
      return MoveStraightTo (*crpiparams_->pose);
      break;
    case CmdMoveThroughTo:
      break;
    case CmdMoveTo:
      if (crpiparams_->moveStraight)
      {
        return MoveStraightTo (*crpiparams_->pose);
      }
      else
      {
        return MoveTo (*crpiparams_->pose);
      }
      break;
    case CmdMoveToAxisTarget:
      return MoveToAxisTarget (*crpiparams_->axes);
      break;
    case CmdRunProgram:
      //! JAM: No CRPI equivalent
      return CANON_REJECT;
      break;
    case CmdSetAbsoluteAcceleration:
      return SetAbsoluteAcceleration (crpiparams_->numPositions);
      break;
    case CmdSetAbsoluteSpeed:
      return SetAbsoluteSpeed (crpiparams_->numPositions);
      break;
    case CmdSetAngleUnits:
      return SetAngleUnits (crpiparams_->str.c_str());
      break;
    case CmdSetAxialSpeeds:
      //! JAM: TODO
      break;
    case CmdSetAxialUnits:
      //! JAM: TODO
      break;
    case CmdSetEndPoseTolerance:
      break;
    case CmdSetIntermediatePoseTolerance:
      break;
    case CmdSetLengthUnits:
      return SetLengthUnits (crpiparams_->str.c_str());
      break;
    case CmdSetParameter:
      //! TODO
//      SetParameter (params_->str.c_str(), *(crpiparams_->numPositions));
      break;
    case CmdSetRelativeAcceleration:
      return SetRelativeAcceleration (crpiparams_->numPositions);
      break;
    case CmdSetRelativeSpeed:
      return SetRelativeSpeed (crpiparams_->numPositions);
      break;
    case CmdSetRobotIO:
      break;
    case CmdSetTool:
      return SetTool (crpiparams_->setting);
      break;
    case CmdStopMotion:
      //! JAM: TODO
      break;
    default:
      return CANON_REJECT;
      break;
    }
     return CANON_SUCCESS;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::CrclXmlResponse (char *str)
  {
    crpiparams_->counter += 1;
    crclxml_->encode(str);
    return CANON_SUCCESS;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::CrpiXmlHandler (std::string& str)
  {
    crpixml_->parse(str); //! Populate the params_ structure based on the XML string

    switch (crpiparams_->cmd)
    {
    case CmdApplyCartesianForceTorque:
      //! TODO
      break;
    case CmdApplyJointTorque:
      //! TODO
      break;
    case CmdCouple:
      return Couple(crpiparams_->str.c_str());
      break;
    case CmdToWorldMatrix:
      break;
    case CmdToSystemMatrix:
      break;
    case CmdFromSystem:
      break;
    case CmdFromWorld:
      break;
    case CmdGetRobotAxes:
      return GetRobotAxes(crpiparams_->axes);
      break;
    case CmdGetRobotForces:
      return GetRobotForces(crpiparams_->forces);
      break;
    case CmdGetRobotIO:
      return GetRobotIO(crpiparams_->io);
      break;
    case CmdGetRobotPose:
      return GetRobotPose(crpiparams_->pose);
      break;
    case CmdGetRobotSpeed:
      //!TODO
      break;
    case CmdGetRobotTorques:
      return GetRobotTorques(crpiparams_->torques);
      break;
    case CmdMessage:
      return Message(crpiparams_->str.c_str());
      break;
    case CmdMoveAttractor:
      return MoveAttractor(*crpiparams_->pose);
      break;
    case CmdMoveStraightTo:
      return MoveStraightTo(*crpiparams_->pose);
      break;
    case CmdMoveThroughTo:
      //! TODO
      break;
    case CmdMoveTo:
      return MoveTo(*crpiparams_->pose);
      break;
    case CmdMoveToAxisTarget:
      return MoveToAxisTarget(*crpiparams_->axes);
      break;
    case CmdSaveConfig:
      return SaveConfig(crpiparams_->str.c_str());
      break;
    case CmdSetAbsoluteAcceleration:
      return SetAbsoluteAcceleration(crpiparams_->numPositions);
      break;
    case CmdSetAbsoluteSpeed:
      return SetAbsoluteSpeed(crpiparams_->numPositions);
      break;
    case CmdSetAngleUnits:
      return SetAngleUnits(crpiparams_->str.c_str());
      break;
    case CmdSetAxialSpeeds:
      //!TODO
      break;
    case CmdSetAxialUnits:
      //!TODO
      break;
    case CmdSetEndPoseTolerance:
      break;
    case CmdSetIntermediatePoseTolerance:
      break;
    case CmdSetLengthUnits:
      return SetLengthUnits(crpiparams_->str.c_str());
      break;
    case CmdSetParameter:
      //SetParameter (params_->str.c_str(), *(params_->numPositions));
      break;
    case CmdSetRelativeAcceleration:
      return SetRelativeAcceleration(crpiparams_->real);
      break;
    case CmdSetRelativeSpeed:
      return SetRelativeSpeed(crpiparams_->real);
      break;
    case CmdSetRobotDO:
      return SetRobotDO(crpiparams_->integer, crpiparams_->boolean);
      break;
    case CmdSetRobotIO:
      //! TODO
      break;
    case CmdSetTool:
      return SetTool(crpiparams_->real);
      break;
    case CmdStopMotion:
      break;
    case CmdToSystem:
      break;
    case CmdToWorld:
      break;
    case CmdUpdateSystemTransform:
      break;
    case CmdUpdateWorldTransform:
      break;
    default:
      return CANON_REJECT;
      break;
    }
     return CANON_SUCCESS;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::CrpiXmlResponse (char *str)
  {
    crpiparams_->counter += 1;
    if (crpixml_->encode(str))
    {
      return CANON_SUCCESS;
    }
    return CANON_FAILURE;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::ToWorldMatrix (matrix & R_T_W)
  {
    R_T_W.resize(4,4);

    for (int iterator1=0; iterator1<4; ++iterator1)
    {
      for (int iterator2=0; iterator2<4; ++iterator2)
      {
        R_T_W.at(iterator1,iterator2) = robotparams_->toWorldMatrix->at(iterator1,iterator2);
      }
    }

    return CANON_SUCCESS;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::ToWorld (robotPose *in, robotPose *out)
  {
    matrix r(3,3);
    matrix mrot(4, 4), mtrans(4, 4), t1(4, 4), t2(4, 4);
    matrix inm(4, 4), outm(4, 4);
    bool flag = true;

#ifdef DOITRIGHTTHISTIME
    t1 = *robotparams_->toWorldMatrix;
#else
    t1 = robotparams_->toWorldMatrix->inv();
#endif
    Math::pose ptemp = in->pose();
    flag &= inm.RPYMatrixConvert(ptemp, (angleUnits_ == DEGREE));
    inm.at(0,3) = in->x;
    inm.at(1,3) = in->y;
    inm.at(2,3) = in->z;
    inm.at(3,3) = 1;
    outm = t1 * inm;
    flag &= outm.matrixRPYConvert(ptemp, (angleUnits_ == DEGREE));
    *out = ptemp;

    if (flag)
    {
      return CANON_SUCCESS;
    }
    else
    {
      return CANON_FAILURE;
    }
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::FromWorld (robotPose *in, robotPose *out)
  {
    matrix r(3, 3);
    matrix mrot(4, 4), mtrans(4, 4), t1(4, 4), t2(4, 4);
    matrix inm(4, 4), outm(4, 4);
    bool flag = true;

#ifdef DOITRIGHTTHISTIME
    t1 = robotparams_->toWorldMatrix->inv();
#else
    t1 = *(robotparams_->toWorldMatrix);
#endif
    //! Convert the robot's rotation to matrix form
    Math::pose ptemp = in->pose();
    flag &= inm.RPYMatrixConvert(ptemp, (angleUnits_ == DEGREE));
    inm.at(0,3) = in->x;
    inm.at(1,3) = in->y;
    inm.at(2,3) = in->z;
    inm.at(3,3) = 1;
    outm = t1 * inm;
    flag &= outm.matrixRPYConvert(ptemp, (angleUnits_ == DEGREE));
    *out = ptemp;
    out->status = in->status;
    out->turns = in->turns;

    if (flag)
    {
      return CANON_SUCCESS;
    }
    else
    {
      return CANON_FAILURE;
    }
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::ToSystem (const char *name,
                                                                     robotPose *in,
                                                                     robotPose *out)
  {
    vector<string>::iterator niter = robotparams_->coordSystNames.begin();
    int pos = 0;
    
    for (; niter != robotparams_->coordSystNames.end(); ++niter, ++pos)
    {
      if (strcmp(niter->c_str(), name) == 0)
      {
        break;
      }
    }
    if (pos >= robotparams_->coordSystNames.size())
    {
      return CANON_FAILURE;
    }

    matrix r(3, 3);
    matrix mrot(4, 4), mtrans(4, 4), t1(4, 4), t2(4, 4);
    matrix inm(4, 4), outm(4, 4);
    bool flag = true;

#ifdef DOITRIGHTTHISTIME
    t1 = *robotparams_->toCoordSystMatrices.at(pos);
#else
    t1 = robotparams_->toCoordSystMatrices.at(pos)->inv();
#endif
    Math::pose ptemp = in->pose();
    flag &= inm.RPYMatrixConvert(ptemp, (angleUnits_ == DEGREE));
  inm.at(0,3) = in->x;
  inm.at(1,3) = in->y;
  inm.at(2,3) = in->z;
  inm.at(3,3) = 1;
    outm = t1 * inm;
    flag &= outm.matrixRPYConvert(ptemp, (angleUnits_ == DEGREE));
    *out = ptemp;

    if (flag)
    {
      return CANON_SUCCESS;
    }
    else
    {
      return CANON_FAILURE;
    }
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::FromSystem (const char *name,
                                                                       robotPose *in,
                                                                       robotPose *out)
  {
    vector<string>::iterator niter = robotparams_->coordSystNames.begin();
    int pos = 0;

    for (; niter != robotparams_->coordSystNames.end(); ++niter, ++pos)
    {
      if (strcmp(niter->c_str(), name) == 0)
      {
        break;
      }
    }
    if (pos >= robotparams_->coordSystNames.size())
    {
      return CANON_FAILURE;
    }

    matrix r(3, 3);
    matrix mrot(4, 4), mtrans(4, 4), t1(4, 4), t2(4, 4);
    matrix inm(4, 4), outm(4, 4);
    bool flag = true;

#ifdef DOITRIGHTTHISTIME
    t1 = robotparams_->toWorldMatrix->inv();
#else
    t1 = *(robotparams_->toCoordSystMatrices.at(pos));
#endif
    //! Convert the robot's rotation to matrix form
    Math::pose ptemp = in->pose();
    flag &= inm.RPYMatrixConvert(ptemp, (angleUnits_ == DEGREE));
  inm.at(0,3) = in->x;
  inm.at(1,3) = in->y;
  inm.at(2,3) = in->z;
  inm.at(3,3) = 1;
    outm = t1 * inm;
    flag &= outm.matrixRPYConvert(ptemp, (angleUnits_ == DEGREE));
    *out = ptemp;

    if (flag)
    {
      return CANON_SUCCESS;
    }
    else
    {
      return CANON_FAILURE;
    }
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::ToSystemMatrix(const char *name, matrix & R_T_W)
  {
    R_T_W.resize(4, 4);

    vector<string>::iterator niter = robotparams_->coordSystNames.begin();
    int pos = 0;

    for (; niter != robotparams_->coordSystNames.end(); ++niter, ++pos)
    {
      if (strcmp(niter->c_str(), name) == 0)
      {
        break;
      }
    }
    if (pos >= robotparams_->coordSystNames.size())
    {
      return CANON_FAILURE;
    }

    for (int iterator1 = 0; iterator1<4; ++iterator1)
    {
      for (int iterator2 = 0; iterator2<4; ++iterator2)
      {
        R_T_W.at(iterator1, iterator2) = robotparams_->toCoordSystMatrices.at(pos)->at(iterator1, iterator2);
      }
    }

    return CANON_SUCCESS;
  }


  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::UpdateWorldTransform(robotPose &newToWorld)
  {
    *(robotparams_->toWorld) = newToWorld;
    Math::pose ptemp = newToWorld.pose();
    if (robotparams_->toWorldMatrix->RPYMatrixConvert(ptemp, (angleUnits_ == DEGREE)))
    {
      return CANON_SUCCESS;
    }

    return CANON_FAILURE;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::UpdateWorldTransform (matrix &newToWorld)
  {
    *(robotparams_->toWorldMatrix) = newToWorld;
    Math::pose ptemp;
    if (robotparams_->toWorldMatrix->matrixRPYConvert(ptemp, (angleUnits_ == DEGREE)))
    {
      *robotparams_->toWorld = ptemp;
      return CANON_SUCCESS;
    }
    return CANON_FAILURE;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::UpdateSystemTransform (const char *name,
                                                                                  robotPose &newToSystem)
  {
    vector<string>::iterator niter = robotparams_->coordSystNames.begin();
    int pos = 0;
    bool flag = false;
    string newname;
    Math::matrix *newmatrix;

    for (; niter != robotparams_->coordSystNames.end(); ++niter, ++pos)
    {
      if (strcmp(niter->c_str(), name) == 0)
      {
        break;
      }
    }
    if (pos >= robotparams_->coordSystNames.size())
    {
      //! Could not find specified system.  A new one is added to the list of heterogeneous transforms.
      flag = true;
      newname = name;
      robotparams_->coordSystNames.push_back(newname);
    }

    if (!flag)
    {
      //! Update existing coordinate system transformation
      robotparams_->toCoordSystPoses.at(pos) = newToSystem;
      Math::pose ptemp = newToSystem.pose();

      if (robotparams_->toCoordSystMatrices.at(pos)->RPYMatrixConvert(ptemp, (angleUnits_ == DEGREE)))
      {
        return CANON_SUCCESS;
      }
      return CANON_FAILURE;
    }
    else
    {
      //! Add new coordinate system transformation
      newmatrix = new Math::matrix(4, 4);
      robotparams_->toCoordSystPoses.push_back(newToSystem);
      Math::pose ptemp = newToSystem.pose();
      if (newmatrix->RPYMatrixConvert(ptemp, (angleUnits_ == DEGREE)))
      {
        robotparams_->toCoordSystMatrices.push_back(newmatrix);
        return CANON_SUCCESS;
      }
      return CANON_FAILURE;
    }
  }

  
  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::UpdateSystemTransform (const char *name,
                                                                                  Math::matrix &newToSystem)
  {
    vector<string>::iterator niter = robotparams_->coordSystNames.begin();
    int pos = 0;
    bool flag = false;
    string newname;
    robotPose newpose;
    Math::matrix *newmatrix;

    for (; niter != robotparams_->coordSystNames.end(); ++niter, ++pos)
    {
      if (strcmp(niter->c_str(), name) == 0)
      {
        break;
      }
    }
    if (pos >= robotparams_->coordSystNames.size())
    {
      //! Could not find specified system.  A new one is added to the list of heterogeneous transforms.
      flag = true;
      newname = name;
      robotparams_->coordSystNames.push_back(newname);
    }

    if (!flag)
    {
      //! Update existing coordinate system transformation 
      *(robotparams_->toCoordSystMatrices.at(pos)) = newToSystem;
      Math::pose ptemp;
      if (robotparams_->toCoordSystMatrices.at(pos)->matrixRPYConvert(ptemp, (angleUnits_ == DEGREE)))
      {
        robotparams_->toCoordSystPoses.at(pos) = ptemp;
        return CANON_SUCCESS;
      }
      return CANON_FAILURE;
    }
    else
    {
      //! Add new coordinate system transformation
      newmatrix = new Math::matrix(4, 4);
      *newmatrix = newToSystem;
      robotparams_->toCoordSystMatrices.push_back(newmatrix);
      Math::pose ptemp;
      if (newmatrix->matrixRPYConvert(ptemp, (angleUnits_ == DEGREE)))
      {
        newpose = ptemp;
        robotparams_->toCoordSystPoses.push_back(newpose);
        return CANON_SUCCESS;
      }
      return CANON_FAILURE;
    }
  }
  

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::SaveConfig (const char *file)
  {
    char line[4096];
    CrpiRobotXml robXML(robotparams_);
    robXML.encode(line);
    ofstream out(file);
    out << line;

    return CANON_SUCCESS;
  }

  template <class T> LIBRARY_API CanonReturn CrpiRobot<T>::IsValid()
  {
    if (bypass_) return CANON_SUCCESS;
    
    if (NULL == robInterface_) return CANON_FAILURE;

    return CANON_SUCCESS;
  }

} // Robot
