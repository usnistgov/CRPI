MODULE CRPI_Handler_Left
  VAR socketdev server_socket;
  VAR socketdev client_socket;
  VAR string receive_string;
  VAR string client_ip;
  VAR bool connected:=FALSE;
  PERS bool recvd_cmd_left:=FALSE;
  PERS bool cmd_done_left:=FALSE;
  PERS bool cmd_okay_left:=FALSE;
  PERS speeddata cmd_speed_Left:=[200,500,200,500];

  VAR num in_arry_left{8};

  ! @brief Main program loop
  !
  PROC CRPI_Main_Left()
    VAR bool aok;
    VAR num value;
    VAR pose lpose;
    VAR pose rpose;
    VAR robtarget r_l_p;
    VAR jointtarget r_l_j;

    VAR num psarry_l{8};
    VAR string tstring1;
    VAR string tstring2;
    VAR string tempstring1;
    VAR string tempstring2;
    VAR num tempnum;
    
    tool0:=GripperL;
    aok:=TRUE;
    TPWrite("Calibrating grippers");
    Hand_Initialize \Calibrate;
    Hand_TurnOffVacuum1;
    
    TPWrite("Running NIST CRPI Server");    
    
    WHILE TRUE DO
      CRPI_GetCmd_Left;
      IF recvd_cmd_left THEN
        
        IF in_arry_left{1} < 100 THEN
          ! Tool command
          IF in_arry_left{1} < 10 THEN
            ! Binary output:  Pneumatic
            IF in_arry_left{2} < 0.5 THEN
              Hand_TurnOffVacuum1;
            ELSE
              Hand_TurnOnVacuum1;
            ENDIF
          ELSEIF in_arry_left{1} < 20 THEN
            ! Analog output:  Servo gripper
            ! use the \holdForce:= option to set the hold force between 0N and 20N
            ! if not used, defaults to 20N
            IF in_arry_left{2} < 0.5 THEN
              Hand_GripInward \NoWait;
            ELSE
              Hand_GripOutward \NoWait;
            ENDIF
          ELSE
              ! Define tool (not used)
          ENDIF
        ELSEIF in_arry_left{1} < 200 THEN
          ! Parameter command
          IF in_arry_Left{1} < 110 THEN
            ! Acceleration
          ELSEIF in_arry_Left{1} < 120 THEN
            ! Speed
            cmd_speed_Left := [in_arry_Left{2}, 500, in_arry_Left{2}, 1000];
          ENDIF          
        ELSEIF in_arry_left{1} < 300 THEN
          ! PTP motion
          IF in_arry_left{1} < 210 THEN
            ! Cartesian
            r_l_p.trans.x := in_arry_left{2};
            r_l_p.trans.y := in_arry_left{3};
            r_l_p.trans.z := in_arry_left{4};
            r_l_p.rot.q1 := in_arry_left{5};
            r_l_p.rot.q2 := in_arry_left{6};
            r_l_p.rot.q3 := in_arry_left{7};
            r_l_p.rot.q4 := in_arry_left{8};
            MoveJ r_l_p, cmd_speed_Left, z50, GripperL;
            
            r_l_p:=CRobT(\TaskRef:=T_ROB_LId\Tool:=GripperL\WObj:=wobj0);
            psarry_l{1}:=1;
            psarry_l{2}:=r_l_p.trans.x;
            psarry_l{3}:=r_l_p.trans.y;
            psarry_l{4}:=r_l_p.trans.z;
            psarry_l{5}:=r_l_p.rot.q1;
            psarry_l{6}:=r_l_p.rot.q2;
            psarry_l{7}:=r_l_p.rot.q3;
            psarry_l{8}:=r_l_p.rot.q4;
          ELSEIF in_arry_left{1} < 220 THEN
            ! Joint
            r_l_j.robax.rax_1 := in_arry_left{2};
            r_l_j.robax.rax_2 := in_arry_left{3};
            r_l_j.extax.eax_a := in_arry_left{4};
            r_l_j.robax.rax_3 := in_arry_left{5};
            r_l_j.robax.rax_4 := in_arry_left{6};
            r_l_j.robax.rax_5 := in_arry_left{7};
            r_l_j.robax.rax_6 := in_arry_left{8};
            MoveAbsJ r_l_j, cmd_speed_Left, z50, GripperL;
            
            r_l_j:=CJointT(\TaskRef:=T_ROB_LId);
            psarry_l{1}:=1;
            psarry_l{2}:=r_l_j.robax.rax_1;
            psarry_l{3}:=r_l_j.robax.rax_2;
            psarry_l{4}:=r_l_j.extax.eax_a;
            psarry_l{5}:=r_l_j.robax.rax_3;
            psarry_l{6}:=r_l_j.robax.rax_4;
            psarry_l{7}:=r_l_j.robax.rax_5;
            psarry_l{8}:=r_l_j.robax.rax_6;
          ELSE 
            ! Force control motion
            ! Not supported
          ENDIF
        ELSEIF in_arry_left{1} < 400 THEN
          ! LIN motion
          ! Cartesian
          r_l_p.trans.x := in_arry_left{2};
          r_l_p.trans.y := in_arry_left{3};
          r_l_p.trans.z := in_arry_left{4};
          r_l_p.rot.q1 := in_arry_left{5};
          r_l_p.rot.q2 := in_arry_left{6};
          r_l_p.rot.q3 := in_arry_left{7};
          r_l_p.rot.q4 := in_arry_left{8};
          MoveL r_l_p, cmd_speed_Left, z50, GripperL;
            
          r_l_p:=CRobT(\TaskRef:=T_ROB_LId\Tool:=GripperL\WObj:=wobj0);
          psarry_l{1}:=1;
          psarry_l{2}:=r_l_p.trans.x;
          psarry_l{3}:=r_l_p.trans.y;
          psarry_l{4}:=r_l_p.trans.z;
          psarry_l{5}:=r_l_p.rot.q1;
          psarry_l{6}:=r_l_p.rot.q2;
          psarry_l{7}:=r_l_p.rot.q3;
          psarry_l{8}:=r_l_p.rot.q4;          
        ELSEIF in_arry_left{1} < 500 THEN
          ! I/O signal
                        
          IF in_arry_left{1} < 410 THEN
            ! Digital
            value := in_arry_left{3};
            IF in_arry_left{2} < 1 THEN
              ! do_0
              SetDO custom_DO_0, value;
            ELSEIF in_arry_left{2} < 2 THEN
              ! out1
              SetDO custom_DO_1, value;
            ELSEIF in_arry_left{2} < 3 THEN
              ! out2
              SetDO custom_DO_2, value;
            ELSEIF in_arry_left{2} < 4 THEN
              ! out3
              SetDO custom_DO_3, value;
            ELSE
              ! out4
              SetDO custom_DO_4, value;
            ENDIF
          ELSE
            ! Analog
            ! Not available
          ENDIF
        ELSEIF in_arry_left{1} < 600 THEN
          ! Cartesian feeddback
          r_l_p:=CRobT(\TaskRef:=T_ROB_LId\Tool:=GripperL\WObj:=wobj0);
          psarry_l{1}:=1;
          psarry_l{2}:=r_l_p.trans.x;
          psarry_l{3}:=r_l_p.trans.y;
          psarry_l{4}:=r_l_p.trans.z;
          psarry_l{5}:=r_l_p.rot.q1;
          psarry_l{6}:=r_l_p.rot.q2;
          psarry_l{7}:=r_l_p.rot.q3;
          psarry_l{8}:=r_l_p.rot.q4;
        ELSEIF in_arry_left{1} < 700 THEN
          ! Joint feedback
          r_l_j:=CJointT(\TaskRef:=T_ROB_LId);
          psarry_l{1}:=1;
          psarry_l{2}:=r_l_j.robax.rax_1;
          psarry_l{3}:=r_l_j.robax.rax_2;
          psarry_l{4}:=r_l_j.extax.eax_a;
          psarry_l{5}:=r_l_j.robax.rax_3;
          psarry_l{6}:=r_l_j.robax.rax_4;
          psarry_l{7}:=r_l_j.robax.rax_5;
          psarry_l{8}:=r_l_j.robax.rax_6;
        ELSEIF in_arry_left{1} < 800 THEN
          ! Cartesian force feedback
        ELSEIF in_arry_left{1} < 900 THEN
          ! Joint torque feedback
          psarry_l{1}:=1;
          psarry_l{2}:=GetMotorTorque(1);
          psarry_l{3}:=GetMotorTorque(2);
          psarry_l{4}:=0;
          psarry_l{5}:=GetMotorTorque(3);
          psarry_l{6}:=GetMotorTorque(4);
          psarry_l{7}:=GetMotorTorque(5);
          psarry_l{8}:=GetMotorTorque(6);   
        ELSEIF in_arry_left{1} < 1000 THEN
          ! I/O feedback
          psarry_l{1}:=1;
          psarry_l{2}:=custom_DI_0;
          psarry_l{3}:=custom_DI_1;
          psarry_l{4}:=custom_DI_2;
          psarry_l{5}:=custom_DI_3;
          psarry_l{6}:=custom_DI_4;
          psarry_l{7}:=custom_DI_5;
          psarry_l{8}:=custom_DI_6;          
        ENDIF

        tstring1:=ValToStr(psarry_l);

        ! Return robot status to the client
        SocketSend client_socket \Str:=tstring1+"\00";
      ENDIF
    ENDWHILE
    
    ERROR
      IF ERRNO=ERR_SOCK_TIMEOUT THEN
        RETRY;
      ELSEIF ERRNO=ERR_SOCK_CLOSED THEN
        RETURN;
      ELSE
        ! No error recovery handling
      ENDIF
  ENDPROC

  
  ! @brief Get commands from the remote client
  !
  PROC CRPI_GetCmd_Left ()
    VAR bool aok;
    
    recvd_cmd_left:=FALSE;
    
    SocketReceive client_socket \Str:=receive_string \Time:=WAIT_MAX;
    
    ! 80 chars max
    aok:=StrToVal(receive_string,in_arry_left);
    IF aok THEN
      !TPWrite("Good string");
    ELSE
      !TPWrite("Bad string");
    ENDIF
    ! [cmdtype, cmdsubtype, 
    
    recvd_cmd_left:=TRUE;

    ERROR
      IF ERRNO = ERR_SOCK_TIMEOUT THEN
        RETRY;
      ELSEIF ERRNO = ERR_SOCK_CLOSED THEN
        CRPI_Recover_Left;
        RETRY;
      ELSE
        ! No error recovery handling
      ENDIF
    UNDO
      SocketClose server_socket;
      SocketClose client_socket;
  ENDPROC

  
  ! @brief Respond to commands from the remote client
  !
  PROC CRPI_RespCmd_Left ()
    VAR bool aok;
    cmd_done_left:=FALSE;
    cmd_okay_left:=FALSE;

    ! in_array_left{1}-in_array_left{8}
    
    SocketReceive client_socket \Str:=receive_string \Time:=WAIT_MAX;
    
    ! 80 chars max
    aok:=StrToVal(receive_string,in_arry_left);
    ! [cmdtype, cmdsubtype, 
    
    recvd_cmd_left:=TRUE;

    ERROR
      IF ERRNO = ERR_SOCK_TIMEOUT THEN
        RETRY;
      ELSEIF ERRNO = ERR_SOCK_CLOSED THEN
        CRPI_Recover_Left;
        RETRY;
      ELSE
        ! No error recovery handling
      ENDIF
    UNDO
      SocketClose server_socket;
      SocketClose client_socket;
  ENDPROC 
  
  
  ! @brief Recover from socket communication errors
  !
  PROC CRPI_Recover_Left()
    TPWrite("CRPI: Waiting for client to connect");
    SocketClose server_socket;
    SocketClose client_socket;
    SocketCreate server_socket;
    SocketBind server_socket, "169.254.152.80", 1025;
    SocketListen server_socket;
    SocketAccept server_socket, client_socket \ClientAddress:=client_ip \Time:=WAIT_MAX;
    TPWrite("CRPI: Client connected");
    ERROR
      IF ERRNO=ERR_SOCK_TIMEOUT THEN
        RETRY;
      ELSEIF ERRNO=ERR_SOCK_CLOSED THEN
        RETURN;
      ELSE
        ! No error recovery handling
      ENDIF
  ENDPROC

ENDMODULE
