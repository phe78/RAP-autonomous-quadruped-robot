//-----------------------------------------------------------------------------------
//   servo control routines
//-----------------------------------------------------------------------------------

void reset_all_legs(int servo_time) {
  int i;
  for (i=0; i<leg_max; i++) reset_leg(i,servo_time);
}

 void zero_leg(int n, int servo_time) {
  int i;
  for (i=0; i <3; i++) {
    joint_angle[n][i]=0;
    move_servo(n,i,servo_time,dont_terminate_command);
  }
  terminate_servo_command();
}

void reset_leg(int n, int servo_time) {

    leg_coor[n][0]=0;
    leg_coor[n][1]=0;
    leg_coor[n][2]=body_heigth;
    move_IK_leg(n,servo_time);
}

void refresh_all_legs(int servo_time) {
  int i;
  for (i=0; i<leg_max; i++) refresh_leg(i,servo_time);
}

void refresh_all_IK_legs(int servo_time) {
  int i;
  for (i=0; i<leg_max; i++) refresh_IK_leg(i,servo_time);
}

void refresh_IK_leg(int n, int servo_time) {
int i;
  for (i=0; i <leg_max; i++) move_IK_leg(i,servo_time);
}

void refresh_leg(int n, int servo_time) {
  int i;
  for (i=0; i <3; i++) move_servo(n,i,servo_time, dont_terminate_command);
  terminate_servo_command();
}

void terminate_servo_command() {
  if (!disable_motors) SSCSerial.println();
  if (debug_mode) Serial.println(); 
}

int  angle_to_pulse(int degree) {
  return((float)degree/servo_angle_max*2000+1500);
}

void move_IK_leg(int l, int dur) {
     if (debug_mode) {
        Serial.print("****** Move IK Leg ");
        Serial.print(l);
        Serial.println(" ******");
        print_float("x ",leg_coor[l][0],8,false);
        print_float("y ",leg_coor[l][1],8,false);        
        print_float("z ",leg_coor[l][2],8,true);
      }
      if (COG_compensation) {
          Serial.print("COG compensation : XCOG=");
          Serial.print(COMP_X);
          Serial.print(" YCOG=");  
          Serial.print(COMP_Y);
          Serial.print(" Zoffset="); 
          Serial.println(leg_Z_offset[l]);                
          IK((leg_coor[l][0]-COMP_X)*leg_direction[l],leg_coor[l][1]-COMP_Y,leg_coor[l][2]+leg_Z_offset[l]);        
      }
      
        else
          IK(leg_coor[l][0]*leg_direction[l],leg_coor[l][1],leg_coor[l][2]);
          
      joint_angle[l][0]=ca*rad_to_deg;
      joint_angle[l][1]=fa*rad_to_deg*leg_direction[l];  
      joint_angle[l][2]=ta*rad_to_deg*leg_direction[l]; 

     if (debug_mode) {
        print_float("ca ",joint_angle[l][0],8,false);
        print_float("fa ",joint_angle[l][1],8,false);        
        print_float("ta ",joint_angle[l][2],8,true);
      }      
      if ((abs(joint_angle[l][0])<=coxa_max_angle)&&(abs(joint_angle[l][1])<=femur_max_angle)&&(abs(joint_angle[l][2])<=tibia_max_angle))
            refresh_leg(l,dur);  
        else print_error("outside range"); 
}
        
void move_servo(int leg_nbr, int joint_nbr, int servo_time, boolean no_ln) {
  
  if (abs(joint_angle[leg_nbr][joint_nbr])<=max_angle[joint_nbr])
     send_pulse(leg_nbr*3+joint_nbr , angle_to_pulse(joint_angle[leg_nbr][joint_nbr])+joint_offset[leg_nbr][joint_nbr], servo_time, no_ln);   
     else print_error("outside range");  
}

void move_servo_offset(int leg_nbr, int joint_nbr, int servo_offset, int servo_time) {

  send_pulse(leg_nbr*3+joint_nbr , joint_offset[leg_nbr][joint_nbr]+pulse_mid , servo_time, terminate_command);
}

void send_pulse(int servo_nbr, int servo_pulse, int servo_time, boolean no_ln) {
  
  if ((servo_pulse<=pulse_max)&&(servo_pulse>=pulse_min)) {
    
    if (!disable_motors) {
      
       SSCSerial.print("#");
       SSCSerial.print(servo_nbr);
       SSCSerial.print("P");
       SSCSerial.print(servo_pulse);
       SSCSerial.print("S");
       if (no_ln) SSCSerial.print(servo_time);
           else SSCSerial.println(servo_time);
    }
       
/*    if (debug_mode) {
       Serial.print("#");
       Serial.print(servo_nbr);
       Serial.print("P");
       Serial.print(servo_pulse);
       Serial.print("S");
       if (no_ln) Serial.print(servo_time);
           else Serial.println(servo_time);
    } */      
  }
}




