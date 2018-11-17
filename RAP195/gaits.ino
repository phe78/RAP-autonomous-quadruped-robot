//-----------------------------------------------------------------------------------
//   Gaits definition
//-----------------------------------------------------------------------------------

#define step_length  70 
#define foot_heigth   60 
#define walk_heigth  310 
#define walk_delay 500 // 1000
// continuous walk steps
#define FF step_length
#define MF step_length/3
#define MR -step_length/3
#define RR -step_length 
// discontinuous walk steps
#define DFF step_length
#define DMF step_length/2
#define DMN 0
#define DMR -step_length/2
#define DRR -step_length 
//
#define DY 40   
#define DZ 10  
#define low_heigth walk_heigth-DZ
#define low_heigth2 walk_heigth-DZ*2

void init_gaits() {

  init_all_seqs();
  
//----------------------------------------------------------------------------------------------------------------------
// Sequence #0 - Push-up
//
// parm index               0                 1                 2                 3                 4               
//                 seq#     X                 Y                 Z                 dZ                dV
//----------------------------------------------------------------------------------------------------------------------
  
  add_seq_step(     0,      0,                 0,               max_heigth,       0,                speed_reduction_ratio,        // Leg#1
                            0,                 0,               max_heigth,       0,                speed_reduction_ratio,        // Leg#2
                            0,                 0,               max_heigth,       0,                speed_reduction_ratio,        // Leg#3
                            0,                 0,               max_heigth,       0,                speed_reduction_ratio);       // Leg#4  
  set_seq_delay(0,2000);      
  add_seq_step(     0,      0,                 0,               default_heigth,   0,                speed_reduction_ratio,        // Leg#1
                            0,                 0,               default_heigth,   0,                speed_reduction_ratio,        // Leg#2
                            0,                 0,               default_heigth,   0,                speed_reduction_ratio,        // Leg#3
                            0,                 0,               default_heigth,   0,                speed_reduction_ratio);       // Leg#4    
  set_seq_delay(0,2000);
  set_seq_loop(0,true); 
//----------------------------------------------------------------------------------------------------------------------
// Sequence #1 - Walk (continuous)
//
// parm index               0                 1                 2                 3                 4               
//                 seq#     X                 Y                 Z                 dZ                dV
//----------------------------------------------------------------------------------------------------------------------
//******1
  add_seq_step(     1,     FF,                 0,               default_heigth,      foot_heigth,                speed_reduction_ratio2,        // Leg#1
                           MR,                 0,               default_heigth,   0,                speed_reduction_ratio2,        // Leg#2
                           MF,                 0,               default_heigth,   0,               speed_reduction_ratio2,        // Leg#3
                           RR,                 0,               default_heigth,   0,                speed_reduction_ratio2);       // Leg#4
  set_seq_delay(1,walk_delay);   
//******2     
  add_seq_step(     1,     MF,                 0,               default_heigth,      0,                speed_reduction_ratio2,        // Leg#1
                           RR,                 0,               default_heigth,   0,                speed_reduction_ratio2,        // Leg#2
                           MR,                 0,               default_heigth,   0,               speed_reduction_ratio2,        // Leg#3
                           FF,                 0,               default_heigth,   foot_heigth,                speed_reduction_ratio2);       // Leg#4
  set_seq_delay(1,walk_delay);      
//******3
  add_seq_step(     1,     MR,                 0,               default_heigth,     0,                speed_reduction_ratio2,        // Leg#1
                           FF,                 0,               default_heigth,   foot_heigth,                speed_reduction_ratio2,        // Leg#2
                           RR,                 0,               default_heigth,   0,               speed_reduction_ratio2,        // Leg#3
                           MF,                 0,               default_heigth,   0,                speed_reduction_ratio2);       // Leg#4
  set_seq_delay(1,walk_delay);   
//******4     
  add_seq_step(     1,     RR,                 0,               default_heigth,      0,                speed_reduction_ratio2,        // Leg#1
                           MF,                 0,               default_heigth,   0,                speed_reduction_ratio2,        // Leg#2
                           FF,                 0,               default_heigth,    foot_heigth,               speed_reduction_ratio2,        // Leg#3
                           MR,                 0,               default_heigth,   0,                speed_reduction_ratio2);       // Leg#4
  set_seq_delay(1,walk_delay);                                
  set_seq_loop(1,true);     
  set_seq_speed(1,default_speed);          
//----------------------------------------------------------------------------------------------------------------------        
//----------------------------------------------------------------------------------------------------------------------
// Sequence #2 - test
//
// parm index               0                 1                 2                 3                 4               
//                 seq#     X                 Y                 Z                 dZ                dV
//----------------------------------------------------------------------------------------------------------------------
  add_seq_step(     2,     RR,                  0,               default_heigth,   0,                speed_reduction_ratio,        // Leg#1
                           FF,                  0,               default_heigth,   0,                speed_reduction_ratio,        // Leg#2
                           MF,                  0,               default_heigth,   0,                speed_reduction_ratio,        // Leg#3
                           RR,                 0,                default_heigth,   0,                speed_reduction_ratio);       // Leg#4  
  set_seq_delay(2,2000);    
  add_seq_step(     2,     FF,                  0,               default_heigth,   0,                speed_reduction_ratio,        // Leg#1
                           RR,                  0,               default_heigth,   0,                speed_reduction_ratio,        // Leg#2
                           MR,                  0,               default_heigth,   0,                speed_reduction_ratio,        // Leg#3
                           FF,                 0,                default_heigth,   foot_heigth,      speed_reduction_ratio);       // Leg#4                                 
  set_seq_delay(2,2000);  
  add_seq_step(     2,     FF,                  0,               default_heigth,   0,                speed_reduction_ratio,        // Leg#1
                           RR,                  0,               default_heigth,   0,                speed_reduction_ratio,        // Leg#2
                           RR,                  0,               default_heigth,   0,                speed_reduction_ratio,        // Leg#3
                           FF,                 0,                default_heigth,   0,                speed_reduction_ratio);       // Leg#4                                 
  set_seq_delay(2,2000);  
  set_seq_loop(2,true); 
  set_seq_speed(2,high_speed);        
//----------------------------------------------------------------------------------------------------------------------       
//----------------------------------------------------------------------------------------------------------------------
// Sequence #3 - initial move
//
// parm index               0                 1                 2                 3                 4               
//                 seq#     X                 Y                 Z                 dZ                dV
//----------------------------------------------------------------------------------------------------------------------
  
  add_seq_step(     3,      0,                 0,               max_heigth,       0,                speed_reduction_ratio,        // Leg#1
                            0,                 0,               max_heigth,       0,                speed_reduction_ratio,        // Leg#2
                            0,                 0,               max_heigth,       0,                speed_reduction_ratio,        // Leg#3
                            0,                 0,               max_heigth,       0,                speed_reduction_ratio);       // Leg#4  
  set_seq_delay(0,1000);      
  add_seq_step(     3,      0,                 0,               default_heigth,   0,                speed_reduction_ratio,        // Leg#1
                            0,                 0,               default_heigth,   0,                speed_reduction_ratio,        // Leg#2
                            0,                 0,               default_heigth,   0,                speed_reduction_ratio,        // Leg#3
                            0,                 0,               default_heigth,   0,                speed_reduction_ratio);       // Leg#4                                 
  set_seq_delay(3,1000);  
  set_seq_loop(3,false);   
  set_seq_speed(3,high_speed);   

//----------------------------------------------------------------------------------------------------------------------
// Sequence #4 - Walk (discontinuous)
//
// parm index               0                 1                 2                 3                 4               
//                 seq#     X                 Y                 Z                 dZ                dV
//----------------------------------------------------------------------------------------------------------------------
//******1

  add_seq_step(     4,     DFF,                 -DY,               default_heigth,      foot_heigth,                speed_reduction_ratio2,        // Leg#1
                           DMN,                 -DY,               default_heigth,   0,                speed_reduction_ratio2,        // Leg#2
                           DMF,                 -DY,               default_heigth,   0,               speed_reduction_ratio2,        // Leg#3
                           DMR,                 -DY,               default_heigth,   0,                speed_reduction_ratio2);       // Leg#4
  set_seq_delay(4,walk_delay);      
  add_seq_step(     4,     DMF,                 0,               default_heigth,    0,                speed_reduction_ratio2,        // Leg#1
                           DMR,                 0,               default_heigth,   0,                speed_reduction_ratio2,        // Leg#2
                           DMN,                  0,               default_heigth,   0,               speed_reduction_ratio2,        // Leg#3
                           DRR,                  0,               default_heigth,   0,                speed_reduction_ratio2);       // Leg#4
  set_seq_delay(4,walk_delay);  
   
//******2  
  add_seq_step(     4,     DMF,                 DY,               default_heigth,      0,                speed_reduction_ratio2,        // Leg#1
                           DMR,                 DY,               default_heigth,   0,                speed_reduction_ratio2,        // Leg#2
                           DMN,                 DY,               default_heigth,   0,               speed_reduction_ratio2,        // Leg#3
                           DFF,                 DY,               default_heigth,   foot_heigth,                speed_reduction_ratio2);       // Leg#4
  set_seq_delay(4,walk_delay);   
  add_seq_step(     4,      DMN,                 DY,               default_heigth-DZ,       0,                speed_reduction_ratio,        // Leg#1
                            DRR,                 DY,               default_heigth,       0,                speed_reduction_ratio,        // Leg#2
                            DMR,                 DY,               default_heigth,       0,                speed_reduction_ratio,        // Leg#3
                            DMF,                 DY,               default_heigth,       0,                speed_reduction_ratio);       // Leg#4  
  set_seq_delay(4,walk_delay);      

//******3
  add_seq_step(     4,     DMN,                 DY,               default_heigth,      0,                speed_reduction_ratio2,        // Leg#1
                           DFF,                 DY,               default_heigth,    foot_heigth,                speed_reduction_ratio2,        // Leg#2
                           DMR,                 DY,               default_heigth,   0,               speed_reduction_ratio2,        // Leg#3
                           DMF,                 DY,               default_heigth,    0,              speed_reduction_ratio2);       // Leg#4
  set_seq_delay(4,walk_delay);   
  add_seq_step(     4,     DMR,                 0,               default_heigth-DZ,       0,                speed_reduction_ratio,        // Leg#1
                            DMF,                 0,               default_heigth,       0,                speed_reduction_ratio,        // Leg#2
                            DRR,                 0,               default_heigth,       0,                speed_reduction_ratio,        // Leg#3
                            DMN,                 0,               default_heigth,       0,                speed_reduction_ratio);       // Leg#4  
  set_seq_delay(4,walk_delay);  

//******4   
  add_seq_step(     4,     DMR,                 -DY,               default_heigth,      0,                speed_reduction_ratio2,        // Leg#1
                           DMF,                 -DY,               default_heigth,   0,               speed_reduction_ratio2,        // Leg#2
                           DFF,                 -DY,               default_heigth,    foot_heigth,                speed_reduction_ratio2,        // Leg#3
                           DMN,                 -DY,               default_heigth,    0,              speed_reduction_ratio2);       // Leg#4
  set_seq_delay(4,walk_delay);   
  add_seq_step(     4,      DRR,                 -DY,               default_heigth-DZ,       0,                speed_reduction_ratio,        // Leg#1
                            DMN,                 -DY,               default_heigth,       0,                speed_reduction_ratio,        // Leg#2
                            DMF,                 -DY,               default_heigth,       0,                speed_reduction_ratio,        // Leg#3
                            DMR,                 -DY,               default_heigth,       0,                speed_reduction_ratio);       // Leg#4  
  set_seq_delay(4,walk_delay);                               
  set_seq_loop(4,true);     
  set_seq_speed(4,high_speed);           

  //----------------------------------------------------------------------------------------------------------------------
// Sequence #5 - Jerk
//
// parm index               0                 1                 2                 3                 4               
//                 seq#     X                 Y                 Z                 dZ                dV
//----------------------------------------------------------------------------------------------------------------------

  add_seq_step(     5,     0,                 -DY,               default_heigth,      0,                speed_reduction_ratio2,        // Leg#1
                           0,                 -DY,               default_heigth,   0,                speed_reduction_ratio2,        // Leg#2
                           0,                 -DY,               default_heigth,   0,               speed_reduction_ratio2,        // Leg#3
                           0,                 -DY,               default_heigth,   0,                speed_reduction_ratio2);       // Leg#4
  set_seq_delay(5,walk_delay);
  add_seq_step(     5,     0,                 0,               default_heigth,      0,                speed_reduction_ratio2,        // Leg#1
                           0,                 0,                default_heigth,   0,                speed_reduction_ratio2,        // Leg#2
                           0,                0,                default_heigth-foot_heigth,   0,       speed_reduction_ratio2,        // Leg#3
                           0,                0,                default_heigth,   0,                speed_reduction_ratio2);       // Leg#4                           
  set_seq_delay(5,walk_delay);
  add_seq_step(     5,     0,                 0,               default_heigth,      0,                speed_reduction_ratio2,        // Leg#1
                           0,                 0,               default_heigth,   0,                speed_reduction_ratio2,        // Leg#2
                           0,                 0,               default_heigth,   0,               speed_reduction_ratio2,        // Leg#3
                           0,                 0,               default_heigth,   0,                speed_reduction_ratio2);       // Leg#4
  set_seq_delay(5,walk_delay);  
  add_seq_step(     5,     0,                 DY,               default_heigth,      0,                speed_reduction_ratio2,        // Leg#1
                           0,                 DY,               default_heigth,   0,                speed_reduction_ratio2,        // Leg#2
                           0,                 DY,               default_heigth,   0,               speed_reduction_ratio2,        // Leg#3
                           0,                 DY,               default_heigth,   0,                speed_reduction_ratio2);       // Leg#4
  set_seq_delay(5,walk_delay);
  add_seq_step(     5,     0,                 0,               default_heigth,      0,                speed_reduction_ratio2,        // Leg#1
                           0,                 0,                default_heigth,   0,                speed_reduction_ratio2,        // Leg#2
                           0,                0,                default_heigth,   0,       speed_reduction_ratio2,        // Leg#3
                           0,                0,                default_heigth-foot_heigth,   0,                speed_reduction_ratio2);       // Leg#4                           
  set_seq_delay(5,walk_delay);
  add_seq_step(     5,     0,                 -DY,               default_heigth,      0,                speed_reduction_ratio2,        // Leg#1
                           0,                 -DY,               default_heigth,   0,                speed_reduction_ratio2,        // Leg#2
                           0,                 -DY,               default_heigth,   0,               speed_reduction_ratio2,        // Leg#3
                           0,                 -DY,               default_heigth,   0,                speed_reduction_ratio2);       // Leg#4
  set_seq_delay(5,walk_delay);
  add_seq_step(     5,     0,                 0,               default_heigth-foot_heigth,      0,                speed_reduction_ratio2,        // Leg#1
                           0,                 0,                default_heigth,   0,                speed_reduction_ratio2,        // Leg#2
                           0,                0,                default_heigth,   0,       speed_reduction_ratio2,        // Leg#3
                           0,                0,                default_heigth,   0,                speed_reduction_ratio2);       // Leg#4                           
  set_seq_delay(5,walk_delay);
  add_seq_step(     5,     0,                 DY,               default_heigth,      0,                speed_reduction_ratio2,        // Leg#1
                           0,                 DY,               default_heigth,   0,                speed_reduction_ratio2,        // Leg#2
                           0,                 DY,               default_heigth,   0,               speed_reduction_ratio2,        // Leg#3
                           0,                 DY,               default_heigth,   0,                speed_reduction_ratio2);       // Leg#4
  set_seq_delay(5,walk_delay);
  add_seq_step(     5,     0,                 0,               default_heigth,      0,                speed_reduction_ratio2,        // Leg#1
                           0,                 0,                default_heigth-foot_heigth,   0,                speed_reduction_ratio2,        // Leg#2
                           0,                0,                default_heigth,   0,       speed_reduction_ratio2,        // Leg#3
                           0,                0,                default_heigth,   0,                speed_reduction_ratio2);       // Leg#4                           
  set_seq_delay(5,walk_delay);   
  set_seq_loop(5,true);   
  //----------------------------------------------------------------------------------------------------------------------
// Sequence #6 - lift leg & COG test
//
// parm index               0                 1                 2                 3                 4               
//                 seq#     X                 Y                 Z                 dZ                dV
//----------------------------------------------------------------------------------------------------------------------  
  add_seq_step(     6,     0,                 0,               default_heigth,      100,                 speed_reduction_ratio2,        // Leg#1
                           0,                 0,               default_heigth,   0,                speed_reduction_ratio2,        // Leg#2
                           0,                0,                default_heigth,   0,       speed_reduction_ratio2,        // Leg#3
                           0,                0,                default_heigth,   0,                speed_reduction_ratio2);       // Leg#4 
  set_seq_delay(6,walk_delay);   
  add_seq_step(     6,     0,                 0,               default_heigth,   0,                 speed_reduction_ratio2,        // Leg#1
                           0,                 0,               default_heigth,   100,                speed_reduction_ratio2,        // Leg#2
                           0,                0,                default_heigth,   0,       speed_reduction_ratio2,        // Leg#3
                           0,                0,                default_heigth,   0,                speed_reduction_ratio2);       // Leg#4 
  set_seq_delay(6,walk_delay);     
  add_seq_step(     6,     0,                 0,               default_heigth,   0,                 speed_reduction_ratio2,        // Leg#1
                           0,                 0,               default_heigth,   0,                speed_reduction_ratio2,        // Leg#2
                           0,                0,                default_heigth,   100,       speed_reduction_ratio2,        // Leg#3
                           0,                0,                default_heigth,   0,                speed_reduction_ratio2);       // Leg#4 
  set_seq_delay(6,walk_delay);  
  add_seq_step(     6,     0,                 0,               default_heigth,   0,                 speed_reduction_ratio2,        // Leg#1
                           0,                 0,               default_heigth,   0,                speed_reduction_ratio2,        // Leg#2
                           0,                0,                default_heigth,   0,       speed_reduction_ratio2,        // Leg#3
                           0,                0,                default_heigth,   100,                speed_reduction_ratio2);       // Leg#4 
  set_seq_delay(6,walk_delay);  
  set_seq_speed(6,low_speed);    
  set_seq_loop(6,false);                                    
//----------------------------------------------------------------------------------------------------------------------           
}

