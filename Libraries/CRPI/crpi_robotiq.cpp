///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Robot Interface
//  Workfile:        CrpiRobotiq.cpp
//  Revision:        1.0 - 13 March, 2014
//  Author:          J. Marvel, J. Falco
//
//  Description
//  ===========
//  CrpiRobotiq interface definitions.
//
///////////////////////////////////////////////////////////////////////////////

#include "crpi_robotiq.h"
#include <iostream>

//#define NOISY

using namespace std;

namespace crpi_robot
{
  void livemanRobotiq (void *param)
  {
    keepalive *ka = (keepalive*)param;
    int val;
    crpi_timer timer;

    while (ka->runThread)
    {
      ((CrpiRobotiq*)ka->rob)->SetParameter("STATUS", &val);

      //! Don't slam your processor!  You don't need to poll at full speed.
      timer.waitUntil(5000);
    }
    return;
  }

  LIBRARY_API CrpiRobotiq::CrpiRobotiq (CrpiRobotParams &params)
  {
    params_ = new CrpiRobotParams();
    *params_ = params;

    action_request = new bitset<8>;
    gripper_options = new bitset<8>;

    for (int i=0; i<43; i++)
    {
      commandRegister_[i] = 0x00;
    }
    commandRegister_[0] = 0x00; // Start Trans ID @ 1
    commandRegister_[1] = 0x01;
    commandRegister_[2] = 0x00; // Protocol ID
    commandRegister_[3] = 0x00;
    commandRegister_[4] = 0x00;  // Use fixed message size to support both 
    commandRegister_[5] = 0x25;
    commandRegister_[6] = 0x02;  // Slave ID
    commandRegister_[7] = 0x10;  // Function code 16 (Preset Multiple Registers)
    commandRegister_[8] = 0x00;  // Address of first register
    commandRegister_[9] = 0x00;
    commandRegister_[10] = 0x00;  // Write all 15 registers
    commandRegister_[11] = 0x0F; 
    commandRegister_[12] = 0x1E;  // Consisting of 30 bytes of data

    statusRegister_[0] = 0x00; // 
    statusRegister_[1] = 0x01;
    statusRegister_[2] = 0x00; // 
    statusRegister_[3] = 0x00;
    statusRegister_[4] = 0x00;  //  
    statusRegister_[5] = 0x06;
    statusRegister_[6] = 0x02;  // 
    statusRegister_[7] = 0x04;  // 
    statusRegister_[8] = 0x00;  // 
    statusRegister_[9] = 0x00;
    statusRegister_[10] = 0x00;  // 
    statusRegister_[11] = 0x0F; 

    //! Establish socket connection
    //clientID_ = ulapi_socket_get_client_id (502, "129.6.35.31"); //"169.254.152.31");
    clientID_ = ulapi_socket_get_client_id (params_->tcp_ip_port, params_->tcp_ip_addr);

#ifdef NOISY
    if (clientID_ < 0)
    {
      cout << "no connection" << endl;
    }
    else
    {
      cout << "connection success" << endl;
    }
#endif
    setHandParam (1, 0);  //Reset Gripper
    
    setHandParam (1, 1);  //Activate Gripper

    getStatusRegisters ();

    PrevFingerA = ReqEcho_PosFingerA;
    PrevFingerB = ReqEcho_PosFingerB;
    PrevFingerC = ReqEcho_PosFingerC;
    PrevScissor = ReqEcho_PosScissor;
    
    grasped_ = false;

    task = ulapi_task_new();
    ka_.handle = ulapi_mutex_new(99);
    ka_.rob = this;
    ka_.runThread = true;

    ulapi_task_start((ulapi_task_struct*)task, livemanRobotiq, &ka_, ulapi_prio_lowest(), 0);
  }

  LIBRARY_API CrpiRobotiq::~CrpiRobotiq ()
  {
    ka_.runThread = false;
  }

  LIBRARY_API CanonReturn CrpiRobotiq::ApplyCartesianForceTorque (robotPose &robotForceTorque, vector<bool> activeAxes, vector<bool> manipulator)
  {
    //! TODO
    return CANON_FAILURE;
  }

  LIBRARY_API CanonReturn CrpiRobotiq::ApplyJointTorque (robotAxes &robotJointTorque)
  {
    //! TODO
    return CANON_FAILURE;
  }

  LIBRARY_API CanonReturn CrpiRobotiq::SetTool (double percent)
  {
    int param;
    if (percent >= 0.5f)
    {
      param = (grasped_) ? 2 : 1;
      grasped_ = false;
    }
    else
    {
      param = 1;
      grasped_ = true;
    }

    setGrip (param);
    return CANON_SUCCESS;
  }

  LIBRARY_API CanonReturn CrpiRobotiq::Couple (const char *targetID)
  {
    int param;
    if (strcmp(targetID, "gripper_gear") == 0)
    {
      param = 1;
      SetParameter("ADVANCED_CONTROL", &param);
      SetParameter("SCISSOR_CONTROL", &param);
      param=100;
      SetParameter("SPEED_FINGER_A", &param);
      SetParameter("SPEED_FINGER_B", &param);
      SetParameter("SPEED_FINGER_C", &param);
      SetParameter("SPEED_SCISSOR", &param);
      param=100;
      SetParameter("FORCE_FINGER_A", &param);
      SetParameter("FORCE_FINGER_B", &param);
      SetParameter("FORCE_FINGER_C", &param);
      SetParameter("FORCE_SCISSOR", &param);
      param=200;
      SetParameter("POSITION_SCISSOR", &param);
      param=10;
      SetParameter("POSITION_FINGER_A", &param);
      param=30;
      SetParameter("POSITION_FINGER_B", &param);
      SetParameter("POSITION_FINGER_C", &param);
      SetTool (0.9);
    }
    else if (strcmp(targetID, "gripper_top_cover") == 0)
    {
      param = 1;
      SetParameter("ADVANCED_CONTROL", &param);
      SetParameter("SCISSOR_CONTROL", &param);
      param=100;
      SetParameter("SPEED_FINGER_A", &param);
      SetParameter("SPEED_FINGER_B", &param);
      SetParameter("SPEED_FINGER_C", &param);
      SetParameter("SPEED_SCISSOR", &param);
      param=100;
      SetParameter("FORCE_FINGER_A", &param);
      SetParameter("FORCE_FINGER_B", &param);
      SetParameter("FORCE_FINGER_C", &param);
      SetParameter("FORCE_SCISSOR", &param);
      param=240;
      SetParameter("POSITION_SCISSOR", &param);
      param=0;
      SetParameter("POSITION_FINGER_A", &param);
      param=30;
      SetParameter("POSITION_FINGER_B", &param);
      SetParameter("POSITION_FINGER_C", &param);
      SetTool (0.9);
    }
    else if (strcmp(targetID, "gripper_bottom_cover") == 0)
    {
      param = 1;
      SetParameter("ADVANCED_CONTROL", &param);
      SetParameter("SCISSOR_CONTROL", &param);
      param=100;
      SetParameter("SPEED_FINGER_A", &param);
      SetParameter("SPEED_FINGER_B", &param);
      SetParameter("SPEED_FINGER_C", &param);
      SetParameter("SPEED_SCISSOR", &param);
      param=100;
      SetParameter("FORCE_FINGER_A", &param);
      SetParameter("FORCE_FINGER_B", &param);
      SetParameter("FORCE_FINGER_C", &param);
      SetParameter("FORCE_SCISSOR", &param);
      param=180;
      SetParameter("POSITION_SCISSOR", &param);
      param=0;
      SetParameter("POSITION_FINGER_A", &param);
      param=30;
      SetParameter("POSITION_FINGER_B", &param);
      SetParameter("POSITION_FINGER_C", &param);
      SetTool (0.9);
    }
    else if (strcmp(targetID, "gripper_finger_test_a") == 0)
    {
      param = 1;
      SetParameter("ADVANCED_CONTROL", &param);
      SetParameter("SCISSOR_CONTROL", &param);
      param=100;
      SetParameter("SPEED_FINGER_A", &param);
      SetParameter("SPEED_FINGER_B", &param);
      SetParameter("SPEED_FINGER_C", &param);
      SetParameter("SPEED_SCISSOR", &param);
      param=100;
      SetParameter("FORCE_FINGER_A", &param);
      SetParameter("FORCE_FINGER_B", &param);
      SetParameter("FORCE_FINGER_C", &param);
      SetParameter("FORCE_SCISSOR", &param);
      param=180;
      SetParameter("POSITION_SCISSOR", &param);
      param=0;
      SetParameter("POSITION_FINGER_A", &param);
      param=30;
      SetParameter("POSITION_FINGER_B", &param);
      SetParameter("POSITION_FINGER_C", &param);
      SetTool (0.9);
    }
      else if (strcmp(targetID, "gripper_finger_test_b") == 0)
    {
      param = 1;
      SetParameter("ADVANCED_CONTROL", &param);
      SetParameter("SCISSOR_CONTROL", &param);
      param=100;
      SetParameter("SPEED_FINGER_A", &param);
      SetParameter("SPEED_FINGER_B", &param);
      SetParameter("SPEED_FINGER_C", &param);
      SetParameter("SPEED_SCISSOR", &param);
      param=100;
      SetParameter("FORCE_FINGER_A", &param);
      SetParameter("FORCE_FINGER_B", &param);
      SetParameter("FORCE_FINGER_C", &param);
      SetParameter("FORCE_SCISSOR", &param);
      param=180;
      SetParameter("POSITION_SCISSOR", &param);
      param=0;
      SetParameter("POSITION_FINGER_A", &param);
      param=30;
      SetParameter("POSITION_FINGER_B", &param);
      SetParameter("POSITION_FINGER_C", &param);
      SetTool (0.9);
    }
    else if (strcmp(targetID, "gripper_finger_test_c") == 0)
    {
      param = 1;
      SetParameter("ADVANCED_CONTROL", &param);
      SetParameter("SCISSOR_CONTROL", &param);
      param=100;
      SetParameter("SPEED_FINGER_A", &param);
      SetParameter("SPEED_FINGER_B", &param);
      SetParameter("SPEED_FINGER_C", &param);
      SetParameter("SPEED_SCISSOR", &param);
      param=100;
      SetParameter("FORCE_FINGER_A", &param);
      SetParameter("FORCE_FINGER_B", &param);
      SetParameter("FORCE_FINGER_C", &param);
      SetParameter("FORCE_SCISSOR", &param);
      param=180;
      SetParameter("POSITION_SCISSOR", &param);
      param=0;
      SetParameter("POSITION_FINGER_A", &param);
      param=30;
      SetParameter("POSITION_FINGER_B", &param);
      SetParameter("POSITION_FINGER_C", &param);
      SetTool (0.9);
    }
    else if (strcmp(targetID, "gripper_peg") == 0)
    {
      param = 1;
      SetParameter("ADVANCED_CONTROL", &param);
      SetParameter("SCISSOR_CONTROL", &param);
      param = 100;
      SetParameter("SPEED_FINGER_A", &param);
      SetParameter("SPEED_FINGER_B", &param);
      SetParameter("SPEED_FINGER_C", &param);
      SetParameter("SPEED_SCISSOR", &param);
      param = 200;
      SetParameter("FORCE_FINGER_A", &param);
      SetParameter("FORCE_FINGER_B", &param);
      SetParameter("FORCE_FINGER_C", &param);
      SetParameter("FORCE_SCISSOR", &param);
      param = 255;
      SetParameter("POSITION_SCISSOR", &param);
      param = 10;
      SetParameter("POSITION_FINGER_A", &param);
      SetParameter("POSITION_FINGER_B", &param);
      SetParameter("POSITION_FINGER_C", &param);
      SetTool(0.9);
    }
    else
    {
      return CANON_FAILURE;
    }
    strcpy (configName, targetID);

    return CANON_SUCCESS;
  }

  LIBRARY_API CanonReturn CrpiRobotiq::Message (const char *message)
  {
    return CANON_SUCCESS;
  }

  LIBRARY_API CanonReturn CrpiRobotiq::MoveStraightTo (robotPose &pose)
  {
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiRobotiq::MoveThroughTo (robotPose *poses,
                                                      int numPoses,
                                                      robotPose *accelerations,
                                                      robotPose *speeds,
                                                      robotPose *tolerances)
  {
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiRobotiq::MoveTo (robotPose &pose)
  {
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiRobotiq::GetRobotAxes (robotAxes *axes)
  {
    //! TODO
    return CANON_FAILURE;
  }


  LIBRARY_API CanonReturn CrpiRobotiq::GetRobotForces (robotPose *forces)
  {
    return CANON_FAILURE;
  }


  LIBRARY_API CanonReturn CrpiRobotiq::GetRobotIO (robotIO *io)
  {
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiRobotiq::GetRobotPose (robotPose *pose)
  {
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiRobotiq::GetRobotSpeed (robotPose *speed)
  {
    return CANON_REJECT;
  }

  
  LIBRARY_API CanonReturn CrpiRobotiq::GetRobotSpeed (robotAxes *speed)
  {
    //! TODO
    return CANON_FAILURE;
  }


  LIBRARY_API CanonReturn CrpiRobotiq::GetRobotTorques (robotAxes *torques)
  {
    //! TODO
    return CANON_FAILURE;
  }


  LIBRARY_API CanonReturn CrpiRobotiq::MoveAttractor (robotPose &pose)
  {
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiRobotiq::MoveToAxisTarget (robotAxes &axes)
  {
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiRobotiq::SetAbsoluteAcceleration (double tolerance)
  {
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiRobotiq::SetAbsoluteSpeed (double speed)
  {
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiRobotiq::SetAngleUnits (const char *unitName)
  {
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiRobotiq::SetAxialSpeeds (double *speeds)
  {
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiRobotiq::SetAxialUnits (const char **unitNames)
  {
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiRobotiq::SetEndPoseTolerance (robotPose &tolerances)
  {
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiRobotiq::SetIntermediatePoseTolerance (robotPose *tolerances)
  {
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiRobotiq::SetLengthUnits (const char *unitName)
  {
    return CANON_SUCCESS;
  }


 LIBRARY_API CanonReturn CrpiRobotiq::SetParameter (const char *paramName, void *paramVal)
  {
    int *temp_int = (int*) paramVal;

    ulapi_mutex_take(ka_.handle);

    if ((strcmp (paramName, "ACTIVATE") == 0))
    {
      setHandParam (1, *temp_int);
    }
    else if (strcmp (paramName, "GRIP_TYPE") == 0)
    {
      setHandParam (2, *temp_int);
    }
    else if (strcmp (paramName, "AUTO_CENTER") == 0)
    {
      setHandParam (4, *temp_int);
    }
    else if (strcmp (paramName, "AUTO_RELEASE") == 0)
    {
      setHandParam (5, *temp_int);
    }
    else if (strcmp (paramName, "ADVANCED_CONTROL") == 0)
    {
      setHandParam (6, *temp_int);
    }
    else if (strcmp (paramName, "SCISSOR_CONTROL") == 0)
    {
      setHandParam (7, *temp_int);
    }
    else if (strcmp (paramName, "POSITION_FINGER_A") == 0)
    {
      setPositionFingerA(*temp_int);
    }
    else if (strcmp (paramName, "SPEED_FINGER_A") == 0)
    {
      setSpeedFingerA(*temp_int);
    }
    else if (strcmp (paramName, "FORCE_FINGER_A") == 0)
    {
      setForceFingerA(*temp_int);
    }
    else if (strcmp (paramName, "POSITION_FINGER_B") == 0)
    {
      setPositionFingerB(*temp_int);
    }
    else if (strcmp (paramName, "SPEED_FINGER_B") == 0)
    {
      setSpeedFingerB(*temp_int);
    }
    else if (strcmp (paramName, "FORCE_FINGER_B") == 0)
    {
      setForceFingerB(*temp_int);
    }
    else if (strcmp (paramName, "POSITION_FINGER_C") == 0)
    {
      setPositionFingerC(*temp_int);
    }
    else if (strcmp (paramName, "SPEED_FINGER_C") == 0)
    {
      setSpeedFingerC(*temp_int);
    }
    else if (strcmp (paramName, "FORCE_FINGER_C") == 0)
    {
      setForceFingerC(*temp_int);
    }
    else if (strcmp (paramName, "POSITION_SCISSOR") == 0)
    {
      setPositionScissor(*temp_int);
    }
    else if (strcmp (paramName, "SPEED_SCISSOR") == 0)
    {
      setSpeedScissor(*temp_int);
    }
    else if (strcmp (paramName, "FORCE_SCISSOR") == 0)
    {
      setForceScissor(*temp_int);
    }
    else if (strcmp (paramName, "GRIP") == 0)
    {
      setGrip(*temp_int);
    }
    else if (strcmp (paramName, "STATUS") == 0)
    {
      getStatusRegisters ();
    }
    else
    {
      return CANON_FAILURE;
    }

    ulapi_mutex_give(ka_.handle);

    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiRobotiq::SetRelativeAcceleration (double percent)
  {
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiRobotiq::SetRelativeSpeed (double percent)
  {
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn CrpiRobotiq::SetRobotIO (robotIO &io)
  {
    //! TODO
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiRobotiq::SetRobotDO (int dig_out, bool val)
  {
    //! TODO
    return CANON_REJECT;
  }


  LIBRARY_API CanonReturn CrpiRobotiq::StopMotion (int condition)
  {
    return CANON_SUCCESS;
  }
  
  LIBRARY_API void CrpiRobotiq::sendCommand()
  {
    int sent = ulapi_socket_write (clientID_, commandRegister_, 43);
    int get;

    get = ulapi_socket_read(clientID_, inbuffer, 8192);
    for (int x = 0; x < get; ++x)
    {
      ackCommand_[x] = (int)inbuffer[x];
      ackCommand_[x] = (ackCommand_[x] < 0)?(ackCommand_[x]+256):(ackCommand_[x]);
    }


    getStatusRegisters ();
  }

  LIBRARY_API void CrpiRobotiq::getStatusRegisters()
  {
    crpi_timer timer;
    timer.waitUntil(200);
    int sent = ulapi_socket_write(clientID_, statusRegister_, 12), get;

    get = ulapi_socket_read (clientID_, inbuffer, 8192);
    {
    for (int i = 0; i < get; ++i)
    {
      ackStatus_[i] = (int)inbuffer[i];
      ackStatus_[i] = (ackStatus_[i] < 0)?(ackStatus_[i]+256):(ackStatus_[i]);
    }

    bitset<8> gripper_status(ackStatus_[9]);

    //extract Initialization Status
    if(gripper_status.test(0)) gACT = true;
    else gACT = false;

    //extract gripper mode 0:Basic 1:Pinch 2:Wide 3:Scissor
    if(!gripper_status.test(1) && !gripper_status.test(2)) gMOD = 0;
    else if (!gripper_status.test(1) && gripper_status.test(2)) gMOD = 1;
    else if (gripper_status.test(1) && !gripper_status.test(2)) gMOD = 2;
    else gMOD = 3;

    //extract Stop/GoTo status
    if(gripper_status.test(3)) gGTO = 1;
    else gGTO = 0;

    //extract setup status
    if(!gripper_status.test(4) && !gripper_status.test(5)) gIMC = 0;
    else if(gripper_status.test(4) && !gripper_status.test(5)) gIMC = 1;
    else if(!gripper_status.test(4) && gripper_status.test(5)) gIMC = 2;
    else  gIMC = 3;

    //extract motion status
    if(!gripper_status.test(6) && !gripper_status.test(7)) gSTA = 0;
    else if(gripper_status.test(6) && !gripper_status.test(7)) gSTA = 1;
    else if(!gripper_status.test(6) && gripper_status.test(7)) gSTA = 2;
    else  gSTA = 3;

    bitset<8> object_status(ackStatus_[10]);

    //extract FingerA status
    if(!object_status.test(0) && !object_status.test(1)) gDTA = 0;
    else if(object_status.test(0) && !object_status.test(1)) gDTA = 1;
    else if(!object_status.test(0) && object_status.test(1)) gDTA = 2;
    else  gDTA = 3;

    //extract FingerB status
    if(!object_status.test(2) && !object_status.test(3)) gDTB = 0;
    else if(object_status.test(2) && !object_status.test(3)) gDTB = 1;
    else if(!object_status.test(2) && object_status.test(3)) gDTB = 2;
    else  gDTB = 3;

    //extract FingerC status
    if(!object_status.test(4) && !object_status.test(5)) gDTC = 0;
    else if(object_status.test(4) && !object_status.test(5)) gDTC = 1;
    else if(!object_status.test(4) && object_status.test(5)) gDTC = 2;
    else  gDTC = 3;

    //extract Scissor status
    if(!object_status.test(6) && !object_status.test(7)) gDTS = 0;
    else if(object_status.test(6) && !object_status.test(7)) gDTS = 1;
    else if(!object_status.test(6) && object_status.test(7)) gDTS = 2;
    else  gDTS = 3;

    //cout << gDTA << " " << gDTB << " " << gDTC  << endl;

    if (gDTA == 1 && gDTB == 1 && gDTC == 1) graspedOnClose_ = true;
    else graspedOnClose_ = false;

    if (gDTA == 2 && gDTB == 2 && gDTC == 2) graspedOnOpen_ = true;
    else graspedOnOpen_ = false;

    if (gDTA == 3 && gDTB == 3 && gDTC == 3) allFingersAtPos_ = true;
    else allFingersAtPos_ = false;

    //extract Fault Status
    gFLT = ackStatus_[11];

    //extract FingerA Stats
    ReqEcho_PosFingerA = ackStatus_[12];
    PosFingerA = ackStatus_[13];
    CurFingerA = ackStatus_[14];

    //extract FingerB Stats
    ReqEcho_PosFingerB = ackStatus_[15];
    PosFingerB = ackStatus_[16];
    CurFingerB = ackStatus_[17];

    //extract FingerC Stats
    ReqEcho_PosFingerC = ackStatus_[18];
    PosFingerC = ackStatus_[19];
    CurFingerC = ackStatus_[20];

    //extract Scissor Stats
    ReqEcho_PosScissor = ackStatus_[21];
    PosScissor = ackStatus_[22];
    CurScissor = ackStatus_[23];
    }
  }


  LIBRARY_API void CrpiRobotiq::setHandParam (int param, int val) 
  {
    //just action request, add Gripper Options Byte1
    bool value;
    unsigned char request;

    if (val == 1) value = true;
    else value = false;

    switch(param)
    {  
      case ACTIVATE: //set rACT to 0:RESET or 1:ACTIVATE
        action_request->set(0,value);  
        request = (unsigned char)action_request->to_ulong();
        commandRegister_[13]=request;
        sendCommand ();
        if (value)
        {
              while (gIMC != 3) getStatusRegisters();
        }
        else
          while (gIMC != 0) getStatusRegisters();
      break;

      case GRIP: 
        setSpeedFingerA(255);
        switch (val)
        {
          case 0: //set rMOD BASIC
          action_request->set(1,false); 
          action_request->set(2,false);
          break;

          case 1: //set rMOD PINCH
          action_request->set(1,false);
          action_request->set(2,true);
          break;

          case 2: //set rMOD WIDE
          action_request->set(1,true);
          action_request->set(2,false);
          break;

          case 3: //set rMOD SCISSOR
          action_request->set(1,true);
          action_request->set(2,true);
          break;

          default:   ;
        }
      request = (unsigned char)action_request->to_ulong();
        commandRegister_[13]=request;
      sendCommand();
          while (gIMC != 3) getStatusRegisters();
            break;

      case MOVE: //set rGTO 0:STOP or 1:GO
        action_request->set(3,value);
        request = (unsigned char)action_request->to_ulong();
          commandRegister_[13]=request;
        sendCommand();
      break;

      case AUTO_RELEASE: //set rSTR 0:NORMAL or 1:AUTO RELEASE
        action_request->set(4,value);
        request = (unsigned char)action_request->to_ulong();
          commandRegister_[13]=request;
        sendCommand();
      break;

      case AUTO_CENTER: //set rAAC 0:NORMAL or 1:AUTO CENTERING
        gripper_options->set(1,value);
        request = (unsigned char)gripper_options->to_ulong();
          commandRegister_[14]=request;
      break;

      case ADVANCED_CONTROL: //set rICF 0:NORMAL or 1:Enable Indvidual Control of Fingers A, B and C
        gripper_options->set(2,value);
        request = (unsigned char)gripper_options->to_ulong();
          commandRegister_[14]=request;
      break;

      case SCISSOR_CONTROL: //set rICS 0:NORMAL 1:Individual SCISSOR Control
        gripper_options->set(3,value);
        request = (unsigned char)gripper_options->to_ulong();
          commandRegister_[14]=request;
      break;

            default:   ;
        } 
  }


  LIBRARY_API int CrpiRobotiq::setGrip(int param)
  {
    int status = 0;
    getStatusRegisters ();

#ifdef NOISY
    writeStatus();
#endif

    if(param == 1)
    { 
      //grasp
      PrevFingerA = ReqEcho_PosFingerA;
      PrevFingerB = ReqEcho_PosFingerB;
      PrevFingerC = ReqEcho_PosFingerC;
      PrevScissor = ReqEcho_PosScissor;

      //writeStatus();

      setHandParam (3,1); //GoTo
      sendCommand();

      clock_t startTime = clock();
      clock_t currentTime;
      double time;

      while (true)
      {
        getStatusRegisters();
        if (gDTA == 3 && gDTB == 3 && gDTC == 3)
        {
          status = 3;
          getStatusRegisters();
          //writeStatus();
          break;
        }
        else if (gDTA == 2 && gDTB == 2 && gDTC == 2)
        {
          status = 2;
          getStatusRegisters();
          break;
        }
        currentTime = clock();
        time = (currentTime-startTime)/(double) CLOCKS_PER_SEC;
        
        if (time > 5) 
        {
          status = 0;
          break;
        }
      }
      //cout << "save: " << PrevFingerA << " " << PrevFingerB << " " << PrevFingerC << endl;
      return status;
    }
    else 
    { //release
      //cout << "release: " << PrevFingerA << " " << PrevFingerB << " " << PrevFingerC << endl;

      setPositionFingerA(PrevFingerA);
      setPositionFingerB(PrevFingerB);
      setPositionFingerC(PrevFingerC);
      setPositionScissor(PrevScissor);

      PrevFingerA = ReqEcho_PosFingerA;
      PrevFingerB = ReqEcho_PosFingerB;
      PrevFingerC = ReqEcho_PosFingerC;
      PrevScissor = ReqEcho_PosScissor;

      setHandParam (3,1);
      sendCommand();

      while (gDTA != 3 && gDTB != 3 && gDTC != 3)
      {
        getStatusRegisters();
      }
      //getStatusRegisters();
      //writeStatus();
    }
    
    return status;
  }


  LIBRARY_API void CrpiRobotiq::writeStatus ()
  {
    cout << endl << "GRIPPER STATUS" << endl;

    cout << "  Initialization: ";

    /*cout << "gACT = " << gACT  << endl;
    cout << "gMOD = " << gMOD  << endl;
    cout << "gGTO = " << gGTO  << endl;
    cout << "gIMC = " << gIMC  << endl;
    cout << "gSTA = " << gSTA  << endl;
    cout << "gDTA = " << gDTA  << endl;
    cout << "gDTB = " << gDTB  << endl;
    cout << "gDTC = " << gDTC  << endl;
    cout << "gDTS = " << gDTS  << endl;
    cout << "gFLT = " << gFLT  << endl;*/

    switch(gACT)
    {  
      case 0: 
        cout << "Gripper reset"<< endl;
      break;
      case 1: 
              cout << "Gripper activation"<< endl;
      break;
      default:   ;
    }

    cout << "  Grasp Mode: ";
    switch(gMOD)
    {  
      case 0:
        cout << "Basic"<< endl;
      break;
      case 1: 
              cout << "Pinch"<< endl;
      break;
      case 2:
        cout << "Wide" << endl;
      break;
      case 3:
        cout << "Scissor" << endl;
      break;
      default:   ;
    }

    cout << "  Stop/Goto: ";
    switch(gGTO)
    {  
      case 0: 
        cout << "Stopped"<< endl;
      break;
      case 1: 
              cout << "GoTo Request"<< endl;
      break;
      default:   ;
    }

    cout << "  Setup: ";
    switch(gIMC)
    {  
      case 0:
        cout << "Reset or auto-release state"<< endl;
      break;
      case 1: 
              cout << "Activation in progress"<< endl;
      break;
      case 2:
        cout << "Mode change in progress" << endl;
        break;
      case 3:
        cout << "Activation/Mode change complete" << endl;
        break;
      default:   
        break;
    }

    cout << "  Motion: ";
    switch(gSTA)
    {  
      case 0:
        cout << "Moving to goal pos"<< endl;
        break;
      case 1: 
        cout << "Stopped (1 or 2 fingers stopped before goal pose)"<< endl;
        break;
      case 2:
        cout << "Stopped (all fingers stopped before goal pose)" << endl;
        break;
      case 3:
        cout << "Stopped (all fingers reached goal pose" << endl;
        break;
      default:
        break;
    }

    cout << endl << "OBJECT STATUS" << endl;

    cout << "  FingerA: ";
    switch(gDTA)
    {  
      case 0:
        cout << "in motion"<< endl;
      break;
      case 1: 
              cout << "stopped (contact on open)"<< endl;
      break;
      case 2:
        cout << "stopped (contact on close)" << endl;
      break;
      case 3:
        cout << "at requested position" << endl;
      break;
      default:   ;
    }

    cout << "  FingerB: ";
    switch(gDTB)
    {  
      case 0:
        cout << "in motion"<< endl;
      break;
      case 1: 
              cout << "stopped (contact on open)"<< endl;
      break;
      case 2:
        cout << "stopped (contact on close)" << endl;
      break;
      case 3:
        cout << "at requested position" << endl;
      break;
      default:   ;
    }

    cout << "  FingerC: ";
    switch(gDTC)
    {  
      case 0:
        cout << "in motion"<< endl;
      break;
      case 1: 
              cout << "stopped (contact on open)"<< endl;
      break;
      case 2:
        cout << "stopped (contact on close)" << endl;
      break;
      case 3:
        cout << "at requested position" << endl;
      break;
      default:   ;
    }

    cout << "  Scissor: ";
    switch(gDTS)
    {  
      case 0:
        cout << "in motion"<< endl;
      break;
      case 1: 
              cout << "stopped (contact on open)"<< endl;
      break;
      case 2:
        cout << "stopped (contact on close)" << endl;
      break;
      case 3:
        cout << "at requested position" << endl;
      break;
      default:   ;
    }

    cout << endl << "OBJECT STATUS" << endl;

    switch(gFLT)
    {  
      case 0:
        cout << "  No Fault"<< endl;
      break;
      case 5: 
              cout << "  Priority Fault: Action delayed, activation(reactivation) must be completed prior to action"<< endl;
      break;
      case 6:
        cout << "  Priority Fault: Action delayed, mode change must be completed prior to action" << endl;
      break;
      case 7:
        cout << "  Priority Fault: The activation bit must be set prior to action" << endl;
      break;
      case 9:
        cout << "  Minor Fault: The communicatin chip is not ready (may be booting)" << endl;
      break;
      case 10:
        cout << "  Minor Fault: Changing mode fault, interferences detected on Scissor (for less than 20 seconds)" << endl;
      break;
      case 11:
        cout << "  Minor Fault: Automatic release in progress" << endl;
      break;
      case 13:
        cout << "  Major Fault: Activation fault, verify that no interference or other error occured" << endl;
      break;
      case 14:
        cout << "  Major Fault: Changing mode fault, interferences detected on Scissor (for more than 20 seconds)" << endl;
      break;
      case 15:
        cout << "  Major Fault: Automatic  release completed.  Reset and activation is required." << endl;
      break;
      default:   ;
    }

    cout << endl << "GRIPPER DATA" << endl;
    cout << "  Finger A" << endl;
    cout << "    Req Echo = " << ReqEcho_PosFingerA << endl;
    cout << "    Position = " << PosFingerA << endl;
    cout << "    Current  = " << CurFingerA << endl;

    cout << "  Finger B" << endl;
    cout << "    Req Echo = " << ReqEcho_PosFingerB << endl;
    cout << "    Position = " << PosFingerB << endl;
    cout << "    Current  = " << CurFingerB << endl;

    cout << "  Finger C" << endl;
    cout << "    Req Echo = " << ReqEcho_PosFingerC << endl;
    cout << "    Position = " << PosFingerC << endl;
    cout << "    Current  = " << CurFingerC << endl;

    cout << "  Scissor" << endl;
    cout << "    Req Echo = " << ReqEcho_PosScissor << endl;
    cout << "    Position = " << PosScissor << endl;
    cout << "    Current  = " << CurScissor << endl;
  }

    void LIBRARY_API CrpiRobotiq::setPositionFingerA(int pose)
  {
    if (pose < 0) pose = 0; // no negative positions
    else if (pose > 255) pose = 255; // max position
    unsigned char y = pose;
    commandRegister_[16]=y;
  }

  void LIBRARY_API CrpiRobotiq::setSpeedFingerA(int speed)
  {
    if (speed < 0) speed = 0; // no negative speeds
    else if (speed > 255) speed = 255; // max speed
    unsigned char y = speed;
    commandRegister_[17]=y;
  }

  void LIBRARY_API CrpiRobotiq::setForceFingerA(int force)
  {
    if (force < 0) force = 0; // no negative force
    else if (force > 255) force = 255; // max force
    unsigned char y = force;
    commandRegister_[18]=y;
  }

  void LIBRARY_API CrpiRobotiq::setPositionFingerB(int pose)
  {
    if (pose < 0) pose = 0; // no negative positions
    else if (pose > 255) pose = 255; // max position
    unsigned char y = pose;
    commandRegister_[19]=y;
  }

  void LIBRARY_API CrpiRobotiq::setSpeedFingerB(int speed)
  {
    if (speed < 0) speed = 0; // no negative speeds
    else if (speed > 255) speed = 255; // max speed
    unsigned char y = speed;
    commandRegister_[20]=y;
  }

  void LIBRARY_API CrpiRobotiq::setForceFingerB(int force)
  {
    if (force < 0) force = 0; // no negative force
    else if (force > 255) force = 255; // max force
    unsigned char y = force;
    commandRegister_[21]=y;
  }

  void LIBRARY_API CrpiRobotiq::setPositionFingerC(int pose)
  {
    if (pose < 0) pose = 0; // no negative positions
    else if (pose > 255) pose = 255; // max position
    unsigned char y = pose;
    commandRegister_[22]=y;
  }

  void LIBRARY_API CrpiRobotiq::setSpeedFingerC(int speed)
  {
    if (speed < 0) speed = 0; // no negative speeds
    else if (speed > 255) speed = 255; // max speed
    unsigned char y = speed;
    commandRegister_[23]=y;
  }

  void LIBRARY_API CrpiRobotiq::setForceFingerC(int force)
  {
    if (force < 0) force = 0; // no negative force
    else if (force > 255) force = 255; // max force
    unsigned char y = force;
    commandRegister_[24]=y;
  }

  void LIBRARY_API CrpiRobotiq::setPositionScissor(int pose)
  {
    if (pose < 0) pose = 0; // no negative positions
    else if (pose > 255) pose = 255; // max position
    unsigned char y = pose;
    commandRegister_[25]=y;
  }

  LIBRARY_API void CrpiRobotiq::setSpeedScissor(int speed)
  {
    if (speed < 0) speed = 0; // no negative speeds
    else if (speed > 255) speed = 255; // max speed
    unsigned char y = speed;
    commandRegister_[26]=y;
  }

  LIBRARY_API void CrpiRobotiq::setForceScissor(int force)
  {
    if (force < 0) force = 0; // no negative speeds
    else if (force > 255) force = 255; // max speed
    unsigned char y = force;
    commandRegister_[27]=y;
  }

} // Robot
