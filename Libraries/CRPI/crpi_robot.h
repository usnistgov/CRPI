///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Robot Interface
//  Workfile:        Robot.h
//  Revision:        1.0 - 11 March, 2014
//                   1.1 - 30 January, 2015 - Updated with formal IO definitions
//                   2.0 - 20 February, 2015 - Transition from CRCL to CRPI.
//                                             Added CRCL XML handler.
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Robot interface declarations.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef crpi_robot_H
#define crpi_robot_H

#include <stddef.h>

#include "crpi.h"
#include "crpi_xml.h"
#include "crpi_robot_xml.h"
#include "vector.h"

using namespace std;
using namespace Xml;
using namespace Math;

namespace crpi_robot
{

  //! @ingroup Robot
  //!
  //! @brief Common template interface for the various robot subtypes
  //!
  template <class T> class LIBRARY_API CrpiRobot
  {
  public:

    //! @brief Default constructor
    //!
    //! @param initPath Path to the file containing the robot's initialization parameters
    //! @param bypass   Whether or not to bypass the actual robot (i.e., to access certain functions
    //!                 actually being connected to a robot)
    //!
    CrpiRobot (const char *initPath, bool bypass = false);

    //! @brief Default destructor
    //!
    ~CrpiRobot ();

    //! @brief Apply a Cartesian Force/Torque at the TCP, expressed in robot base coordinate system
    //!
    //! @param robotForceTorque The Cartesian command forces and torques applied at the end-effector
    //!        activeAxes       Toggle which axes will be slated for active force control.
    //!                         TRUE = ACTIVE, FALSE = INACTIVE
    //!        manipulator      Toggle which manipulators will be slated for active force control.
    //1                         TRUE = ACTIVE, FALSE = INACTIVE (useful for hands)
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn ApplyCartesianForceTorque (robotPose &robotForceTorque,
                                           vector<bool> activeAxes,
                                           vector<bool> manipulator);

    //! @brief Apply joint torques
    //!
    //! @param robotJointTorque are the command torques for the respective joint axes
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn ApplyJointTorque (robotAxes &robotJointTorque);

    //! @brief Dock with a specified target object
    //!
    //! @param targetID The name of the object with which the robot should dock
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn Couple (const char *targetID);

    //! @brief Get feedback from the robot regarding its current axis configuration
    //!
    //! @param axes Axis array to be populated by the method
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn GetRobotAxes (robotAxes *axes);

    //! @brief Get the measured Cartesian forces from the robot
    //!
    //! @param forces Cartesian force data structure to be populated by the method
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn GetRobotForces (robotPose *forces);

    //! @brief Get I/O feedback from the robot
    //!
    //! @Param io Digital and analog I/O data structure to be populated by the method
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn GetRobotIO (robotIO *io);

    //! @brief Get feedback from the robot regarding its current position in Cartesian space
    //!
    //! @param pose Cartesian pose data structure to be populated by the method
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn GetRobotPose (robotPose *pose);

    //! @brief Get instantaneous Cartesian velocity
    //!
    //! @param speed Cartesian velocities to be populated by the method
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn GetRobotSpeed (robotPose *speed);

    //! @brief Get instantaneous joint speeds
    //!
    //! @param speed Joint velocities array to be populated by the method
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn GetRobotSpeed (robotAxes *speed);

    //! @brief Get joint torques from the robot regarding
    //!
    //! @param torques Axis array to be populated by the method
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn GetRobotTorques (robotAxes *torques);

    //! @brief Display a message on the operator console
    //!
    //! @param message The plain-text message to be displayed on the operator console
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn Message (const char *message);
    
    //! @brief Move a virtual attractor to a specified coordinate in Cartesian space for force control
    //!
    //! @param pose The 6DOF destination of the virtual attractor 
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn MoveAttractor (robotPose &pose);

    //! @brief Move the robot in a straight line from the current pose to a new pose and stop there
    //!
    //! @param pose The target 6DOF pose for the robot
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn MoveStraightTo (robotPose &pose);

    //! @brief Move the controlled point along a trajectory passing through or near all but the last
    //!        of a series of poses, and then stop at the last pose
    //!
    //! @param poses         An array of 6DOF poses through/near which the robot is expected to pass
    //! @param numPoses      The number of sub-poses in the submitted array
    //! @param accelerations (optional) An array of 6DOF accelaration profiles for each motion
    //!                      associated with the target poses
    //! @param speeds        (optional) An array of 6DOF speed profiles for each motion assiciated
    //!                      with the target poses
    //! @param tolerances    (optional) An array of 6DOF tolerances in length and angle units for the
    //!                      specified target poses
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    //! @note The length of the optional parameter arrays, if provided, must be equal to numPoses.
    //! @note Defining accerlations, speeds, and tolerances does not overwrite the defined default
    //!       values
    //!
    CanonReturn MoveThroughTo (robotPose *poses,
                               int numPoses,
                               robotPose *accelerations = NULL,
                               robotPose *speeds = NULL,
                               robotPose *tolerances = NULL);

    //! @brief Move the controlled pose along any convenient trajectory from the current pose to the
    //!        target pose, and then stop.
    //!
    //! @param pose The target 6DOF Cartesian pose for the robot's TCP in Cartesian space coordinates
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn MoveTo (robotPose &pose);

    //! @brief Move the robot axes to the specified target values
    //!
    //! @param axes An array of target axis values specified in the current axial unit
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn MoveToAxisTarget (robotAxes &axes);

    //! @brief Set the acceleration for the controlled pose to the given value in length units per
    //!        second per second
    //!
    //! @param acceleration The target TCP acceleration 
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn SetAbsoluteAcceleration (double acceleration);

    //! @brief Set the speed for the controlled pose to the given value in length units per second
    //!
    //! @param speed The target Cartesian speed
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn SetAbsoluteSpeed (double speed);

    //! @brief Set angel units to the unit specified
    //!
    //! @param unitName The name of the angle units in plain text ("degree" or "radian")
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn SetAngleUnits (const char *unitName);

    //! @brief Set the axis-specific speeds for the motion of axis-space motions
    //!
    //! @param speeds Array of target axial motion speeds
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn SetAxialSpeeds (double *speeds);

    //! @brief Set specific axial units to the specified values
    //!
    //! @param unitNames Array of axis-specific names of the axis units in plain text
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn SetAxialUnits (const char **unitNames);

    //! @brief Set the default 6DOF tolerances for the pose of the robot in current length and angle
    //!        units
    //!
    //! @param tolerances Tolerances of the 6DOF end pose during Cartesian motion commands
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn SetEndPoseTolerance (robotPose &tolerance);

    //! @brief Set the default 6DOF tolerance for smooth motion near intermediate points
    //!
    //! @param tolerances Tolerances of the 6DOF poses during multi-pose motions
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn SetIntermediatePoseTolerance (robotPose *tolerances);

    //! @brief Set length units to the unit specified
    //!
    //! @param unitName The name of the length units in plain text ("inch," "mm," and "meter")
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn SetLengthUnits (const char *unitName);

    //! @brief Set a robot-specific parameter (handling of parameter type casting to be handled by the
    //!        robot interface)
    //!
    //! @param paramName The name of the parameter variable to set
    //! @param paramVal  The value to be set to the specified robot parameter
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn SetParameter (const char *paramName, void *paramVal);

    //! @brief Set the accerlation for the controlled pose to the given percentage of the robot's
    //!        maximum acceleration
    //!
    //! @param percent The percentage of the robot's maximum acceration in the range of [0, 1]
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn SetRelativeAcceleration (double percent);

    //! @brief Set the speed for the controlled point to the given percentage of the robot's maximum
    //!        speed
    //!
    //! @param percent The percentage of the robot's maximum speed in the range of [0, 1]
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn SetRelativeSpeed (double percent);

    //! @brief Set the digital and analog outputs
    //!
    //! @param io Digital and analog I/O outputs to set
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn SetRobotIO (robotIO &io);

    //! @brief Set a specific digital output
    //!
    //! @param dig_out Digital output channel to set
    //! @param val     Value to set the digital output
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn SetRobotDO (int dig_out, bool val);

    //! @brief Set the attached tool to a defined output rate.
    //!
    //! @param percent The desired output rate for the robot's tool as a percentage of maximum output.
    //!                If using a gripper, 0 is all the way open, 1 is all the way closed.
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn SetTool (double percent);

    //! @brief Stop the robot's motions based on robot stopping rules
    //!
    //! @param condition The rule by which the robot is expected to stop (Estop category 0, 1, or 2);
    //!                  Estop category 2 is default
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn StopMotion (int condition = 2);

    CanonReturn SetJointType(int index, CanonJointType type);
    CanonReturn GetJointType(int index, CanonJointType *type);

    //! @brief Convert CRCL XML to CRPI function calls
    //!
    //! @param str CRCL XML string to be interpreted as a CRPI function call
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn CrclXmlHandler (std::string &str);

    //! @brief Generate a status message for a CRCL client
    //!
    //! @param str Response CRCL XML message to be sent to the CRCL client
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn CrclXmlResponse (char *str);

    //! @brief Convert CRPI XML to CRPI function calls
    //!
    //! @param str CRPI XML string to be interpreted as a function call
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn CrpiXmlHandler (std::string &str);

    //! @brief Generate a status message for a CRPI XML client
    //!
    //! @param str Response CRPI XML message to be sent to the client
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn CrpiXmlResponse (char *str);

    //! @brief Populates a reference to a matrix object with the transformation matrix from robot to world
    //!
    //! @param R_T_W Matrix object representing the transformation from the robot coordinate system to
    //!              world populated by this function
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn ToWorldMatrix(matrix & R_T_W);

    //! @brief Populate a matrix with the transformation matrix from the robot to a specified coordinate system
    //!
    //! @param name  The name of the coordinate system being referenced
    //! @param R_T_S Transformation matrix, populated by this function, representing the transformation from
    //!              the robot to the specified coordinate system
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn ToSystemMatrix(const char *name, matrix &R_T_S);

    //! @brief Project the robot's pose into world coordinates
    //!
    //! @param in  The target pose of the robot in the robot's coordinate frame
    //! @param out The target pose of the robot in the world coordinate frame
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn ToWorld (robotPose *in, robotPose *out);

    //! @brief Project a world coordinate frame pose into the robot's coordinate frame
    //!
    //! @param in  The target pose of the robot in the world coordinate frame
    //! @param out The target pose of the robot in the robot's base coordinate frame
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn FromWorld (robotPose *in, robotPose *out);
    
    //! @brief Project the robot's pose into a specified coordinate system's coordinates
    //!
    //! @param name The name of the specified coordinate system
    //! @param in   The target pose of the robot in the robot's coordinate frame
    //! @param out  The target pose of the robot in the world coordinate frame
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn ToSystem(const char *name, robotPose *in, robotPose *out);

    //! @brief Project a specified coordinate system's pose into the robot's coordinate frame
    //!
    //! @param name The name of the specified coordinate system
    //! @param in   The target pose of the robot in the world coordinate frame
    //! @param out  The target pose of the robot in the robot's base coordinate frame
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn FromSystem(const char *name, robotPose *in, robotPose *out);

    //! @brief Overwrite the transformation from the robot's coordinate frame to the world coordinate frame
    //!
    //! @param newToSystem The updated transformation from robot to world
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn UpdateWorldTransform (robotPose &newToSystem);

    //! @brief Overwrite the transformation from the robot's coordinate frame to the world coordinate frame
    //!
    //! @param newToWorld The updated transformation from robot to world
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn UpdateWorldTransform (matrix &newToSystem);

    //! @brief Overwrite the transformation from the robot's coordinate frame to a specified coordinate system
    //!
    //! @param newToWorld The updated transformation from robot to world
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    //! @note If the specified system transformation does not exist, a new rigid transformation with the
    //!       given name is added to the robot's configuration.
    //!
    CanonReturn UpdateSystemTransform(const char *name, robotPose &newToWorld);

    //! @brief Overwrite the transformation from the robot's coordinate frame to a specified coordinate system
    //!
    //! @param name        The name of the coordinate system to which we are providing an updated transformation
    //! @param newToWorld The updated transformation from robot to world
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    //! @note If the specified system transformation does not exist, a new rigid transformation with the
    //!       given name is added to the robot's configuration.
    //!
    CanonReturn UpdateSystemTransform(const char *name, matrix &newToWorld);

    //! @brief Save the robot configuration parameters to disk
    //!
    //! @param file The destination location for the new configuration file
    //!
    //! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
    //!         not accepted, and FAILURE if the command is accepted but not executed successfully
    //!
    CanonReturn SaveConfig (const char *file);

    /* FMP */
    //! @brief Checks that the robot has benen fully initialized
    //!
    //! @return true if initialized, else false
    CanonReturn IsValid();

  private:
    //! @brief Interface object for the different supported robots
    //!
    T *robInterface_;

    //! @brief CRPI function parameters for interpretation of XML-based commands
    //!
    CrpiXmlParams *crpiparams_;

    //! @brief Settings for connecting to and communicating with a remote robot
    //!
    CrpiRobotParams *robotparams_;

    //! @brief Handler for interpreting CRCL XML commands as CRPI C++ function calls
    //!
    CrclXml *crclxml_;

    //! @brief Handler for interpreting CRPI XML commands as C++ function calls
    //!
    CrpiXml *crpixml_;

    //! @brief Variables used (and abused) throughout the CrpiRobot class for rotation representation
    //!        conversions.  Added here for memory efficiency.
    matrix *rotMatrix_;
    vector3D *v1_;
    vector3D *v2_;
    vector3D *v3_;

    //! @brief Units by which orientations are reported
    //!
    CanonAngleUnit angleUnits_;

    //! @brief Units by which locations and Euclidean distances are reported
    //!
    CanonLengthUnit lengthUnits_;

    //! @brief Whether or not to run this in bypass mode
    //!
    bool bypass_;

    //! @brief True if the robot has been fully initialized
    bool valid_;
    
  }; // CrpiRobot
} // crpi_robot

#endif
