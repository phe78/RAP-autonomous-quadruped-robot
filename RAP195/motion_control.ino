//-----------------------------------------------------------------------------------
//   motion control routines
//-----------------------------------------------------------------------------------

void lift_leg(int ln, int lh) {
  compute_COG(ln);
  compensate_COG();  
  leg_coor[ln][2]=lh;
  move_IK_leg(ln,low_speed);
}

void down_leg(int ln) {
  leg_coor[ln][2]=default_heigth;
  move_IK_leg(ln,low_speed);
  restore_COG();  
}


void compute_COG(int lift_leg) {

  int leg_Z;

  switch (lift_leg) {
    case 0:   // C,A,B=1,2,3
      C[0]=leg_coor[1][0]+bl/2;
      C[1]=leg_coor[1][1]+bw/2;      
      A[0]=leg_coor[2][0]-bl/2;
      A[1]=leg_coor[2][1]-bw/2;    
      B[0]=leg_coor[3][0]-bl/2;
      B[1]=leg_coor[3][1]+bw/2; 
      leg_Z=3;  
      break;          
    case 1:   // C,A,B=0,2,3  
      C[0]=leg_coor[0][0]+bl/2;
      C[1]=leg_coor[0][1]-bw/2;      
      A[0]=leg_coor[2][0]-bl/2;
      A[1]=leg_coor[2][1]-bw/2;    
      B[0]=leg_coor[3][0]-bl/2;
      B[1]=leg_coor[3][1]+bw/2;
      leg_Z=2;        
      break;        
    case 2:   // C,A,B=0,1,3    
      C[0]=leg_coor[0][0]+bl/2;
      C[1]=leg_coor[0][1]-bw/2;      
      A[0]=leg_coor[1][0]+bl/2;
      A[1]=leg_coor[1][1]+bw/2;    
      B[0]=leg_coor[3][0]-bl/2;
      B[1]=leg_coor[3][1]+bw/2;
      leg_Z=1;        
      break;       
    case 3:   // C,A,B=0,1,2     
      C[0]=leg_coor[0][0]+bl/2;
      C[1]=leg_coor[0][1]-bw/2;      
      A[0]=leg_coor[1][0]+bl/2;
      A[1]=leg_coor[1][1]+bw/2;    
      B[0]=leg_coor[2][0]-bl/2;
      B[1]=leg_coor[2][1]-bw/2;
      leg_Z=0;        
      break;       
  }
  K[0]=(C[0]+A[0])/2;
  K[1]=(C[1]+A[1])/2;
  I[0]=(A[0]+B[0])/2;
  I[1]=(A[1]+B[1])/2;  
  J[0]=(B[0]+C[0])/2;
  J[1]=(B[1]+C[1])/2;  
  DKL3=(B[1]-K[1])/(B[0]-K[0]);
  DJL2=(A[1]-J[1])/(A[0]-J[0]);
  COG_coor[0]=(J[1]-K[1]-J[0]*DJL2+K[0]*DKL3)/(DKL3-DJL2);
  COG_coor[1]=(COG_coor[0]-J[0])*DJL2+J[1];
  DCOG=sqrtf(COG_coor[0]*COG_coor[0]+COG_coor[1]*COG_coor[1]);
  BETA=(DCOG-SCOG)/DCOG;

/*  Serial.print(" A = ");
  Serial.println(A[0]); 
  Serial.println(A[1]); 
  Serial.print(" B = ");
  Serial.println(B[0]); 
  Serial.println(B[1]);  
  Serial.print(" C = ");
  Serial.println(C[0]); 
  Serial.println(C[1]);    

  Serial.print(" I = ");
  Serial.println(I[0]); 
  Serial.println(I[1]); 

  Serial.print(" J = ");
  Serial.println(J[0]); 
  Serial.println(J[1]); 
 
  Serial.print(" K = ");
  Serial.println(K[0]); 
  Serial.println(K[1]);    

  Serial.print(" DKL3 = ");
  Serial.println(DKL3);   

  Serial.print(" DJL2 = ");
  Serial.println(DJL2);      */

  Serial.print("XCOG = ");
  Serial.print(COG_coor[0]);
  Serial.print(" YCOG = ");
  Serial.println(COG_coor[1]);  

/*  Serial.print(" BETA = ");
  Serial.println(BETA);     */

  switch (lift_leg) {
    case 0 ... 1: 
      COMP_X=0;
      COMP_Y=COG_coor[1]*BETA;
      break;
    case 2 ... 3:
//      COMP_X=COG_coor[0]*BETA;
      COMP_X=0;      
      COMP_Y=COG_coor[1]*BETA;   
      break;
   default:
      break;
   }
   for (int i=0;i<leg_max;i++)
    if (i==leg_Z) leg_Z_offset[i]=COG_Z_compensation;
      else leg_Z_offset[i]=0;
}

void compensate_COG() {
  delay(COG_delay);  
  COG_compensation = true;
  refresh_all_IK_legs(low_speed2);
  delay(COG_delay);
}

void restore_COG() {
  delay(COG_delay);   
  COG_compensation = false; 
  refresh_all_IK_legs(low_speed2);  
  delay(COG_delay); 
}

void do_step(int seq_nbr, int step_nbr, int segments, int move_speed) {

  int i,l;
  float x0,y0,z0,x1,y1,z1,dz,dv;

  for (l=0; l<leg_max;l++) {
    
      motion_step[0][l][0]=leg_coor[l][0];
      motion_step[0][l][1]=leg_coor[l][1];
      motion_step[0][l][2]=leg_coor[l][2];
      motion_step[0][l][3]=move_speed/motion_seq[seq_nbr][step_nbr][l][4];    
      
      motion_step[segments][l][0]=motion_seq[seq_nbr][step_nbr][l][0];
      motion_step[segments][l][1]=motion_seq[seq_nbr][step_nbr][l][1];
      motion_step[segments][l][2]=motion_seq[seq_nbr][step_nbr][l][2];
      motion_step[segments][l][3]=move_speed/motion_seq[seq_nbr][step_nbr][l][4];        
  }
  
  for (l=0;l<leg_max;l++) {
       x0=leg_coor[l][0];
       y0=leg_coor[l][1];
       z0=leg_coor[l][2];
       x1=motion_seq[seq_nbr][step_nbr][l][0];
       y1=motion_seq[seq_nbr][step_nbr][l][1];
       z1=motion_seq[seq_nbr][step_nbr][l][2];
       dz=motion_seq[seq_nbr][step_nbr][l][3]; 
       dv=motion_seq[seq_nbr][step_nbr][l][4];  

       if (dz>0)  {
        compute_COG(l);
        compensate_COG();
       }
       
       for (i=1; i<segments; i++) {
         motion_step[i][l][0]=x0+(x1-x0)*i/segments;
         motion_step[i][l][1]=y0+(y1-y0)*i/segments;
         motion_step[i][l][2]=z0+(z1-z0)*i/segments; 
         if (dz>0) motion_step[i][l][2]-=dz*sin(PI*i/segments);  
         motion_step[i][l][3]=int(move_speed/(dv-(dv-1)*sin(PI*i/segments)));
       }

  }

  for (i=0; i<=segments; i++) {   
      for (l=0;l<leg_max;l++) {
        leg_coor[l][0]=motion_step[i][l][0];
        leg_coor[l][1]=motion_step[i][l][1];  
        leg_coor[l][2]=motion_step[i][l][2]; 
        if (debug_mode) {
/*              Serial.print("segment ");
              Serial.println(i);              
              print_float("x ", leg_coor[l][0],8,false);
              print_float("y ", leg_coor[l][1],8,false);
              print_float("z ", leg_coor[l][2],8,false);  
              print_float("s ", motion_step[i][l][3],8,true); */
        } 
        move_IK_leg(l,motion_step[i][l][3]);  
      }
     if (!disable_motors) wait_motion_complete();  
        else delay(100);            
  }
  
  if (COG_compensation) restore_COG();
       
}

void wait_motion_complete() {
  boolean mc;
  String  st;
  mc = false;
  
  while (!mc) {           // first wait for movement start "+"
    SSCSerial.println("Q");
    st=SSCSerial.read();
    mc = (st == "+");
  }
  mc=false;
  while (!mc) {
    SSCSerial.println("Q");
    st=SSCSerial.read();
    mc = (st == ".");
  }
}

void init_seq(int seq_nbr) {
  last_step[seq_nbr]=0;
  motion_seq[seq_nbr][0][0][4]=end_seq_tag;
  sequ_speed[seq_nbr]=default_speed;
}

void init_all_seqs() {
  int i;
  for (i=0;i<max_seq;i++) init_seq(i);
}

void add_seq_step(int seq_nbr, float x1, float y1, float z1, float dz1, int dv1,
                               float x2, float y2, float z2, float dz2, int dv2,
                               float x3, float y3, float z3, float dz3, int dv3,
                               float x4, float y4, float z4, float dz4, int dv4) {

  int n=last_step[seq_nbr];
 
  motion_seq[seq_nbr][n][0][0]=x1;  
  motion_seq[seq_nbr][n][0][1]=y1;
  motion_seq[seq_nbr][n][0][2]=z1; 
  motion_seq[seq_nbr][n][0][3]=dz1; 
  motion_seq[seq_nbr][n][0][4]=dv1; 
 
  motion_seq[seq_nbr][n][1][0]=x2;  
  motion_seq[seq_nbr][n][1][1]=y2;
  motion_seq[seq_nbr][n][1][2]=z2; 
  motion_seq[seq_nbr][n][1][3]=dz2; 
  motion_seq[seq_nbr][n][1][4]=dv2; 
 
  motion_seq[seq_nbr][n][2][0]=x3;  
  motion_seq[seq_nbr][n][2][1]=y3;
  motion_seq[seq_nbr][n][2][2]=z3; 
  motion_seq[seq_nbr][n][2][3]=dz3; 
  motion_seq[seq_nbr][n][2][4]=dv3; 
 
  motion_seq[seq_nbr][n][3][0]=x4;  
  motion_seq[seq_nbr][n][3][1]=y4;
  motion_seq[seq_nbr][n][3][2]=z4; 
  motion_seq[seq_nbr][n][3][3]=dz4; 
  motion_seq[seq_nbr][n][3][4]=dv4;     

  next_step(seq_nbr);
}

void set_seq_loop(int seq_nbr, boolean loop_switch) {

  if (loop_switch) motion_seq[seq_nbr][last_step[seq_nbr]][0][4]=loop_seq_tag;
    else motion_seq[seq_nbr][last_step[seq_nbr]][0][4]=end_seq_tag;
  next_step(seq_nbr);
}

void set_seq_wait(int seq_nbr) {

  motion_seq[seq_nbr][last_step[seq_nbr]][0][4]=wait_move_tag;
  next_step(seq_nbr); 
}


void set_seq_delay(int seq_nbr, int seq_delay) {

  motion_seq[seq_nbr][last_step[seq_nbr]][0][4]=delay_seq_tag;
  motion_seq[seq_nbr][last_step[seq_nbr]][0][0]=seq_delay;
  next_step(seq_nbr); 
}

void set_seq_speed(int seq_nbr, int spd) {

  sequ_speed[seq_nbr]=spd;
}

void next_step(int seq_nbr) {
  last_step[seq_nbr]++;
}

void run_seq(int seq_nbr) {
  int i;
  boolean loop_seq;

  Serial.print("********* run_seq ");
  Serial.print(seq_nbr); 
  Serial.println(" ******");

  Serial.print("last_step= ");
  Serial.println(last_step[seq_nbr]); 
  
  loop_seq = true;

  while (digitalRead(clearButton) == LOW);

  while (loop_seq) {
    
   for (i=0;i<=last_step[seq_nbr];i++) {
    
      Serial.print("######### step ");
      Serial.println(i);
      Serial.print("######### tag ");
      Serial.println(motion_seq[seq_nbr][i][0][4]);
    
      switch (motion_seq[seq_nbr][i][0][4]) {
          
        case 1 ... 16: // valid range of speed reduction ratio
           Serial.println("do step"); 
           do_step(seq_nbr, i,default_segments,sequ_speed[seq_nbr]);
           break;
        case wait_move_tag:
           wait_motion_complete();
           break;
        case end_seq_tag:
           Serial.println("end");           
           loop_seq=false;
           break;
        case delay_seq_tag:
           Serial.print("delay : ");
           Serial.println(motion_seq[seq_nbr][i][0][0]);        
           delay(motion_seq[seq_nbr][i][0][0]);
           break;
        case loop_seq_tag:
           Serial.println("loop");        
           loop_seq=true;
          break;
        } // switch

       if (digitalRead(clearButton) == LOW ) {
          short_tone();
          i=last_step[seq_nbr]-1;
          loop_seq=false;         
          while (digitalRead(clearButton) == LOW);
          
        } // if
        
   }// for 
   
  } // while

  body_heigth=default_heigth;
  reset_all_legs(default_speed);
  
}


