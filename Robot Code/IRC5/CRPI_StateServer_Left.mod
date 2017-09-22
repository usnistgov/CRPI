MODULE CRPI_StateServer_Left
  VAR socketdev state_server_socket;
  VAR socketdev state_client_socket;
  VAR string state_client_ip;
  VAR bool state_connected:=FALSE;

  ! @brief Main program loop
  !
  PROC CRPI_State_Left()
    VAR robtarget r_l_p;
    VAR jointtarget r_l_j;
    VAR num psarry_l{8};
    VAR string tstring;
        
    TPWrite("Running Left NIST CRPI State Server ");
    WHILE TRUE DO
      ! ---------------------------------------------------------------  
      !                        Cartesian Feedback
      ! ---------------------------------------------------------------
      r_l_p:=CRobT(\TaskRef:=T_ROB_LId \Tool:=tool0 \WObj:=wobj0);
      psarry_l{1}:=1;
      psarry_l{2}:=Trunc(r_l_p.trans.x\Dec:=5);
      psarry_l{3}:=Trunc(r_l_p.trans.y\Dec:=5);
      psarry_l{4}:=Trunc(r_l_p.trans.z\Dec:=5);
      psarry_l{5}:=Trunc(r_l_p.rot.q1\Dec:=5);
      psarry_l{6}:=Trunc(r_l_p.rot.q2\Dec:=5);
      psarry_l{7}:=Trunc(r_l_p.rot.q3\Dec:=5);
      psarry_l{8}:=Trunc(r_l_p.rot.q4\Dec:=5);
      tstring:=ValToStr(psarry_l);
      ! Return robot status to the client
      SocketSend state_client_socket \Str:=tstring+"\00";

      ! ---------------------------------------------------------------  
      !                          Joint Feedback
      ! ---------------------------------------------------------------
      r_l_j:=CJointT(\TaskRef:=T_ROB_LId);
      psarry_l{1}:=2;
      psarry_l{2}:=Trunc(r_l_j.robax.rax_1\Dec:=5);
      psarry_l{3}:=Trunc(r_l_j.robax.rax_2\Dec:=5);
      psarry_l{4}:=Trunc(r_l_j.extax.eax_a\Dec:=5);
      psarry_l{5}:=Trunc(r_l_j.robax.rax_3\Dec:=5);
      psarry_l{6}:=Trunc(r_l_j.robax.rax_4\Dec:=5);
      psarry_l{7}:=Trunc(r_l_j.robax.rax_5\Dec:=5);
      psarry_l{8}:=Trunc(r_l_j.robax.rax_6\Dec:=5);
      tstring:=ValToStr(psarry_l);
      ! Return robot status to the client
      SocketSend state_client_socket \Str:=tstring+"\00";

      ! ---------------------------------------------------------------  
      !                       Joint Torque Feedback
      ! ---------------------------------------------------------------
      psarry_l{1}:=3;
      psarry_l{2}:=Trunc(GetMotorTorque(1)\Dec:=5);
      psarry_l{3}:=Trunc(GetMotorTorque(2)\Dec:=5);
      psarry_l{4}:=0;
      psarry_l{5}:=Trunc(GetMotorTorque(3)\Dec:=5);
      psarry_l{6}:=Trunc(GetMotorTorque(4)\Dec:=5);
      psarry_l{7}:=Trunc(GetMotorTorque(5)\Dec:=5);
      psarry_l{8}:=Trunc(GetMotorTorque(6)\Dec:=5);
      tstring:=ValToStr(psarry_l);
      ! Return robot status to the client
      SocketSend state_client_socket \Str:=tstring+"\00";

      ! ---------------------------------------------------------------  
      !                           Digital Inputs 
      ! ---------------------------------------------------------------
      psarry_l{1}:=4;
      psarry_l{2}:=custom_DI_0;
      psarry_l{3}:=custom_DI_1;
      psarry_l{4}:=custom_DI_2;
      psarry_l{5}:=custom_DI_3;
      psarry_l{6}:=custom_DI_4;
      psarry_l{7}:=custom_DI_5;
      psarry_l{8}:=custom_DI_6;

      tstring:=ValToStr(psarry_l);
      ! Return robot status to the client
      SocketSend state_client_socket \Str:=tstring+"\00";

      WaitTime 0.03; ! 30 Hz
    ENDWHILE
    
    ERROR
      IF ERRNO=ERR_SOCK_TIMEOUT THEN
        RETRY;
      ELSEIF ERRNO=ERR_SOCK_CLOSED THEN
        CRPI_Recover_State_Left;
        RETRY;
      ELSE
        ! No error recovery handling
      ENDIF
  ENDPROC

  
  ! @brief Recover from socket communication errors
  !
  PROC CRPI_Recover_State_Left()
    TPWrite("CRPI: Waiting for client to connect to state server");
    SocketClose state_server_socket;
    SocketClose state_client_socket;
    SocketCreate state_server_socket;
    SocketBind state_server_socket, "169.254.152.80", 2025;
    SocketListen state_server_socket;
    SocketAccept state_server_socket, state_client_socket \ClientAddress:=state_client_ip \Time:=WAIT_MAX;
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

