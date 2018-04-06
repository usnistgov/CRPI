///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       CRPI Test
//  Workfile:        Source.cpp
//  Revision:        1.0 23 July, 2014
//					 2.0 20 March, 2018  Linux compatibility added
//  Author:          J. Marvel - Edited by M. Zimmerman
//
//  Description
//  ===========
//  Test program for running CRPI with basic functionality using a UR5 robot.
///////////////////////////////////////////////////////////////////////////////




#include <iostream>
#include <time.h>
#ifdef WIN32
#include <stdlib.h>
#include "crpi_robot.h"
#include "crpi_universal.h"
#include "ulapi.h"
#else
#include "../../Libraries/CRPI/crpi_robot.h"
#include "../../Libraries/CRPI/crpi_universal.h"
#include "../../Libraries/ulapi/src/ulapi.h"
#endif

using namespace crpi_robot;
using namespace std;

void main()
{
	//! Create the robot object
	CrpiRobot<CrpiUniversal> arm("universal_ur5.xml");

	//! Configure the default units
	arm.SetAngleUnits("degree");
	arm.SetLengthUnits("mm");

	//! Couple the robot with the parallel gripper tool
	arm.Couple("gripper_parallel");

	cout << "Robot connected" << endl;

	//! Temporary storage for current and target poses
	robotPose curPose, newPose;

	//! Get the current pose of the robot
	arm.GetRobotPose(&curPose);
	curPose.print();

	//! Set a new pose target for 10 mm higher than the current position
	newPose = curPose;
	newPose.z += 10.0f;

	cout << "Moving robot up by 10 mm" << endl;
	//! Move robot
	if (arm.MoveStraightTo(newPose) == CANON_SUCCESS)
	{
		//! Display the updated position
		arm.GetRobotPose(&curPose);
		curPose.print();
	}
	else
	{
		//! Oops, something went wrong
		cout << "Could not move robot." << endl;
	}
	cout << "Complete" << endl;
}
