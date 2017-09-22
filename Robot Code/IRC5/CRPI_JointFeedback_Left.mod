MODULE CRPI_StateServer_Left

  LOCAL VAR CRPI_socket sock;
  LOCAL VAR CRPI_message mssg;

  ! @brief Main program loop
  !
  PROC main()
    sock.port := 1025;
    
    TPWrite("Running NIST CRPI State Server");
    
    WHILE TRUE DO
      ! ---------------------------------------------------------------  
      !                  Cartesian Feedback Command
      ! ---------------------------------------------------------------
      r_l_p:=CRobT(\TaskRef:=T_ROB_LId \Tool:=GripperL \WObj:=wobj0);
      mssg.arry{1}:=1;
      mssg.arry{2}:=r_l_p.trans.x;
      mssg.arry{3}:=r_l_p.trans.y;
      mssg.arry{4}:=r_l_p.trans.z;
      mssg.arry{5}:=r_l_p.rot.q1;
      mssg.arry{6}:=r_l_p.rot.q2;
      mssg.arry{7}:=r_l_p.rot.q3;
      mssg.arry{8}:=r_l_p.rot.q4;
      CRPI_SendMssg sock, mssg;

      ! ---------------------------------------------------------------  
      !                    Joint Feedback Command
      ! ---------------------------------------------------------------
      r_l_j:=CJointT(\TaskRef:=T_ROB_LId);
      mssg.arry{1}:=1;
      mssg.arry{2}:=r_l_j.robax.rax_1;
      mssg.arry{3}:=r_l_j.robax.rax_2;
      mssg.arry{4}:=r_l_j.extax.eax_a;
      mssg.arry{5}:=r_l_j.robax.rax_3;
      mssg.arry{6}:=r_l_j.robax.rax_4;
      mssg.arry{7}:=r_l_j.robax.rax_5;
      mssg.arry{8}:=r_l_j.robax.rax_6;
      CRPI_SendMssg sock, mssg;

      ! ---------------------------------------------------------------  
      !                  Joint Torque Feedback Command
      ! ---------------------------------------------------------------
      mssg.arry{1}:=1;
      mssg.arry{2}:=GetMotorTorque(1);
      mssg.arry{3}:=GetMotorTorque(2);
      mssg.arry{4}:=0;
      mssg.arry{5}:=GetMotorTorque(3);
      mssg.arry{6}:=GetMotorTorque(4);
      mssg.arry{7}:=GetMotorTorque(5);
      mssg.arry{8}:=GetMotorTorque(6);   
      CRPI_SendMssg sock, mssg;

      ! ---------------------------------------------------------------  
      !                 Digital Input Feedback Command
      ! ---------------------------------------------------------------
      mssg.arry{1}:=1;
      mssg.arry{2}:=custom_DI_0;
      mssg.arry{3}:=custom_DI_1;
      mssg.arry{4}:=custom_DI_2;
      mssg.arry{5}:=custom_DI_3;
      mssg.arry{6}:=custom_DI_4;
      mssg.arry{7}:=custom_DI_5;
      mssg.arry{8}:=custom_DI_6;          
      CRPI_SendMssg sock, mssg;
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

ENDMODULE
