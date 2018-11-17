
/*
-----------------------------------------------------------------------------------
 RAP Robotic Autonomous Pet 
 V1.95

 (c) Philippe Elie

 Release history

  1.95 Novc 11, 2018
  - COG compensation

  1.80 Oct 28, 2018
  - LIDAR mode
  
  1.70 Oct 25, 2018
  - leg sensors 
  - LIDAR
  
  1.60 Oct 18, 2018
  - sensors mode
  - leg direction
 
 1.50 Oct 15, 2018
  - new motion sequencer

 1.40 Oct 14, 2018
  - run mode

 1.30 Oct 14, 2018
  - wifi support
 
 1.10 Oct 12, 2018
  - curved trajectories
  - motion acceleration / deceleration
  - motion sequencer
 
 1.00 Oct 9, 2018
  - servo control mode 
  - offset control mode
  - cartesian IK mode
  - Lynxmotion SSC32U servo controller
  
  leg offsets 
  L1  -21 49 -29
  L2  100 87 35
  L3  61 -59 -35
  L4  32 45 19
-----------------------------------------------------------------------------------  
*/

#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <string.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <YDLidar.h>
#include "QueueList.h"

YDLidar lidar;
LiquidCrystal_I2C lcd(0x3F,20,4);
LSM6    gyro_acc;
LIS3MDL mag;
IntervalTimer leg_sensor_timer;

#define rap_version           "1.95"

#define SSCSerial             Serial2
#define wifiSerial            Serial3
#define LIDARSerial           Serial4

#define debug_flag            true
#define disable_motors_flag   false
//-----------------------------------------------------------------------------------
//   LIDAR
//-----------------------------------------------------------------------------------
#define SIZE_OF_SCAN_BUFFER 32
bool isScanning = false, LIDAR_on;  
#define YDLIDAR_MOTOR_SCTP 35 // The PWM pin for control the speed of YDLIDAR's motor. 
#define YDLIDAR_MOTRO_EN   36 // The ENABLE PIN for YDLIDAR's motor  
unsigned int LIDAR_dist[360];
unsigned int min_dist,max_dist,min_dist_angle,max_dist_angle;
QueueList<scanPoint> scans;      
//-----------------------------------------------------------------------------------
//   IMU
//-----------------------------------------------------------------------------------
#define max_dev   4 
#define GRAVITY 256 
#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second
#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi
#define M_X_MIN -7209 // -1000
#define M_Y_MIN -5013 //1000
#define M_Z_MIN -4274 // -1000
#define M_X_MAX -1104 // +1000
#define M_Y_MAX 937 //+1000
#define M_Z_MAX 2188 // +1000
/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data,
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1
#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw
float pitch, roll, yaw;
char report[80];
LIS3MDL::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};
// X axis pointing forward, Y axis pointing to the left, and Z axis pointing up.
// Positive pitch : nose down , Positive roll : right wing down , Positive yaw : counterclockwise
int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors
int gyro_x, gyro_y,gyro_z;
int accel_x,accel_y,accel_z;
int magnetom_x,magnetom_y,magnetom_z;
float c_magnetom_x,c_magnetom_y,c_magnetom_z;
float MAG_Heading;
float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};
long timer=0;   //general purpuse timer
long timer_old;
unsigned int counter=0;
float G_Dt=0.02;  
byte gyro_sat=0;
float errorRollPitch[3]= {0,0,0};
float errorYaw[3]= {0,0,0};
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here
float DCM_Matrix[3][3]= {
   { 1,0,0 }
  ,{ 0,1,0 }
  ,{ 0,0,1 }
};
float Temporary_Matrix[3][3]={
   { 0,0,0 }
  ,{ 0,0,0 }
  ,{ 0,0,0 }
};
//-----------------------------------------------------------------------------------
//   IK & geometry
//-----------------------------------------------------------------------------------
#define leg_max    4
#define leg_dof    3
#define cl  55.5    // coxa length
#define fl  134.8   // femur length
#define tl  139.4   // tibia length
#define bw  320     // footprint width
#define bl  320     // footprint lentgh
#define max_heigth      330
#define min_heigth      130
#define default_heigth  310
#define rad_to_deg      180/PI
#define coxa_max_angle  80
#define femur_max_angle 80
#define tibia_max_angle 113
int     max_angle[3]={coxa_max_angle, femur_max_angle, tibia_max_angle}; 
float   ll,a1,a2,fa,b1,ta,x1,z2,z3,ca,z4,z5,z6,z1,y2,y11;
float   body_heigth;
signed  int leg_direction[4]={-1,-1,-1,-1};
//-----------------------------------------------------------------------------------
//   pin definitions
//-----------------------------------------------------------------------------------
int  encoderPinA = 2;   
int  encoderPinB = 3;
int  clearButton = 4;
int  buzzerpin   = 11;
int  keyboardpin = 22;
int  leg_sensor_pin[4] = {14, 15, 16, 17};
//-----------------------------------------------------------------------------------
//   servo control
//-----------------------------------------------------------------------------------

#define servo_angle_max       270
#define pulse_max             2500
#define pulse_min             500
#define pulse_mid             (pulse_max+pulse_min)/2
#define offset_max            200
//-----------------------------------------------------------------------------------
//   leg sensors
//-----------------------------------------------------------------------------------
int leg_sensor_threshold[4]={200,500,400,400};
volatile int leg_sensor[4],xsens;
volatile boolean leg_on_ground[4];
#define LEG_SENSOR_INTERRUPT_DELAY 200000
#define LEG_THRESHOLD 200
//-----------------------------------------------------------------------------------
//   keyboard
//-----------------------------------------------------------------------------------
#define key_previous_page     1
#define key_next_page         2
#define key_previous_parm     3
#define key_next_parm         4
#define key_exec              5

#define delay_key             50
#define keyb_range            2

int keyboard_table[]={1023,1,111, 272, 754, 786};
volatile signed int encoderPos = 0;  // a counter for the dial
signed int lastReportedPos = 1;   // change management
static boolean rotating = false;    // debounce management
boolean A_set = false;
boolean B_set = false;
//-----------------------------------------------------------------------------------
//   wifi
//-----------------------------------------------------------------------------------
bool wifi_DEBUG = true;   //show more logs
int  responseTime = 10; //   wifi communication timeout
const String  WifiNetwork="\"HomeExt\"",WifiKey="\"PoM62m63N91\"";
String IP_address="";
//-----------------------------------------------------------------------------------
//   edit modes & system variables
//-----------------------------------------------------------------------------------
#define EEPROM_offset_address 0
#define splash_delay          1000
#define edit_leg_mode         0
#define edit_offset_mode      1
#define edit_IK_mode          2
#define run_mode              3
#define sensors_mode          4
#define LIDAR_mode            5
int edit_state, current_edit_leg, current_edit_parm;
boolean debug_mode,disable_motors;
int change_page,x,x0;

signed int joint_angle[leg_max][leg_dof] = //  [leg=FL,LR,RL,RR] [joint = coxa, femur, tibia]
{
  {0,0,0}, // FL
  {0,0,0}, // FR2
};
signed int joint_offset[leg_max][leg_dof] = //  [leg=FL,LR,RL,RR] [joint = coxa, femur, tibia]
{
  {0,0,0}, // FL
  {0,0,0}, // FR
  {0,0,0}, // RL
  {0,0,0}  // RR
}; //
float leg_coor[leg_max][3] = //  [leg=FL,LR,RL,RR] [X,Y,Z]
{
  {0,0,0}, // FL
  {0,0,0}, // FR
  {0,0,0}, // RL
  {0,0,0}  // RR
}; //
float leg_Z_offset[leg_max] = { 0, 0, 0, 0} ; // Z offset for pitch/roll correction
//-----------------------------------------------------------------------------------
//   COG compensation
//-----------------------------------------------------------------------------------
float COG_coor[2];
int SCOG = 10;  // stability area radius around COG
float A[2],B[2],C[2],I[2],J[2],K[2],DKL3, DJL2,DCOG,BETA,COMP_X,COMP_Y;
boolean COG_compensation = false;
#define COG_delay 200
#define COG_Z_compensation -20
#define lift_heigth 250
//-----------------------------------------------------------------------------------
//   motion
//-----------------------------------------------------------------------------------
#define default_speed         4000
#define speed_reduction_ratio 5
#define speed_reduction_ratio2 8
#define low_speed             500
#define low_speed2            200
#define high_speed            20000
#define max_segments          20
#define default_segments      16
#define max_step              128
#define max_seq               8
#define terminate_command     false
#define dont_terminate_command true
//-----------------------------------------------------------------------------------
//   motion sequencer variables
//      opcode=motion_seq[max_seq][max_step][0][4] : >=0 means speed ratio ;  -1 means end of seq ; -2 means loop to beginning ; -3 means delay
//       parm1=motion_seq[max_seq][max_step][l][0] : x if opcode >=0 ; delay duration if Int 0 = -3 ; max loop if Int 0 = -2
//       parm2=motion_seq[max_seq][max_step][l][1] : y if opcode >=0 ; current loop counter if Int 0 = -2
//       parm3=motion_seq[max_seq][max_step][l][2] : z if opcode >=0 
//       parm4=motion_seq[max_seq][max_step][l][3] : dz if opcode >=0  (curved path on Z axis)
//       parm5=motion_seq[max_seq][max_step][l][4] : dv if opcode >=0  (acceleration / deceleration ratio)
//-----------------------------------------------------------------------------------
#define end_seq_tag       -1
#define loop_seq_tag      -2
#define delay_seq_tag     -3
#define wait_move_tag     -4
int     motion_seq[max_seq][max_step][leg_max][5];    // 5 = X, Y, Z, dZ, dV
float   motion_step[max_segments][leg_max][4];        // 4 = X, Y, Z, V
int     last_step[max_seq],sequ_speed[max_seq],current_seq;
//-----------------------------------------------------------------------------------
//   setup
//-----------------------------------------------------------------------------------
void setup() {
  
  Serial.begin(9600);  
  SSCSerial.begin(115200);
  wifiSerial.begin(115200); 

  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(clearButton, INPUT);
  pinMode(buzzerpin,  OUTPUT);  
  
  digitalWrite(encoderPinA, HIGH);
  digitalWrite(encoderPinB, HIGH);
  digitalWrite(clearButton, HIGH);
  attachInterrupt(encoderPinA, doEncoderA, CHANGE);
  attachInterrupt(encoderPinB, doEncoderB, CHANGE);  
  
  Wire.begin();
  lcd.init();
  lcd.backlight();
  splash_screen();

  short_tone();  
  
  LIDAR_init();
  imu_init();  
  
  load_offset_values();
  clear_IK_values();
  current_edit_leg = 0;  // leg = 0 = FL

  COG_compensation = false;
  COMP_X=0;
  COMP_Y=0;
  
  edit_state = run_mode;
  debug_mode = debug_flag;
  disable_motors = disable_motors_flag;

  init_gaits();
  display_seqs();

  wifi_init();  
  display_IP_address(IP_address);

  init_leg_sensors();  

  run_seq(3);

  init_mode();

//  calibrate_mag();
}  
//-----------------------------------------------------------------------------------
//   loop
//-----------------------------------------------------------------------------------
void loop() {

  String message; 
    
     
      if (LIDAR_on) read_LIDAR();
                  
      rotating = true;  // reset the debouncer
      
      if (lastReportedPos != encoderPos) {
        lastReportedPos = encoderPos;
        switch (edit_state) {
           case edit_leg_mode:
             display_leg_values_update_parm(encoderPos); 
             break;
           case edit_offset_mode:
             display_offset_values_update_parm(encoderPos);
             break;
           case edit_IK_mode:
             display_IK_values_update_parm(encoderPos);
             break;      
           case run_mode:
             display_run_values_update_parm(encoderPos);
             break;    
           case sensors_mode:
             break;    
        } // switch (edit_state)       
      } // if

      if (edit_state==sensors_mode)  {
         read_imu();
//         if ((abs(int(ToDeg(pitch)))>max_dev) || (abs(int(ToDeg(roll)))>max_dev)) set_leg_Z_offset(-pitch, -roll);  
         display_sensors_values();
      }
      
      if (edit_state==LIDAR_mode) display_LIDAR_values();  

      if ((wifiSerial.available()>0)&&(edit_state==run_mode)) {
          message = readWifiSerialMessage();
          message=parse_received_message(message);
          Serial.print("wifi command:");
          Serial.println(message);
          short_tone();
          if (message=="H") {
             body_heigth=max_heigth;
             reset_all_legs(default_speed);
             }
            else if (message=="L") {
                body_heigth=default_heigth;
                reset_all_legs(default_speed);
                 }      
            else if (message=="S") run_seq(current_seq); 
            else if  (message=="H0") lift_leg(0,lift_heigth);
            else if  (message=="H1") lift_leg(1,lift_heigth);
            else if  (message=="H2") lift_leg(2,lift_heigth);
            else if  (message=="H3") lift_leg(3,lift_heigth);                                                                       
            else if  (message=="L0") down_leg(0);  
            else if  (message=="L1") down_leg(1);          
            else if  (message=="L2") down_leg(2);  
            else if  (message=="L3") down_leg(3);                      
        } // if
  
     x = read_keyboard();
     
     if (x>0) {
         x0=x;
         delay(delay_key);
         
         if (read_keyboard()==x0) {
             tick_tone();
             switch(x) {
               case key_exec :                  //---------- EXEC key ----------
                  switch (edit_state) {
                     case edit_leg_mode:
                       lastReportedPos = 0;
                       encoderPos = 0;
                       display_leg_values_update_parm(encoderPos);
                       break;
                     case edit_offset_mode:
                       save_offset_values();
                       load_offset_values();            
                       short_tone();
                       break;
                     case edit_IK_mode:
                       clear_IK_values();
                       display_IK_values_titles();
                       display_IK_values();         
                       short_tone();
                       break;      
                     case run_mode:
                       short_tone();
                       run_seq(current_seq);
                       break;    
                     case LIDAR_mode:
                       toggle_LIDAR_status();
                       break;                           
                     case sensors_mode:
                       break;    
                  } // switch (edit_state) 
                  break;   
               case key_next_parm :               //---------- NEXT PARM key ---------- 
                  switch (edit_state) {
                     case edit_leg_mode:
                       if (current_edit_parm>=3) current_edit_parm=0;
                         else  current_edit_parm++;
                       change_edit_leg_parm();                        
                       break;
                     case edit_offset_mode:
                       if (current_edit_parm>=3) current_edit_parm=0;
                         else  current_edit_parm++;
                       change_edit_offset_parm();                      
                       break;
                     case edit_IK_mode:
                       if (current_edit_parm>=3) current_edit_parm=0;
                         else  current_edit_parm++;
                       change_edit_IK_parm();                      
                       break;      
                     case run_mode:
                       if (current_edit_parm>=1) current_edit_parm=0;
                         else  current_edit_parm++;
                       change_edit_run_parm();                     
                       break;    
                     case sensors_mode:
                       break;    
                     case LIDAR_mode:
                       dump_LIDAR_dist();
                       break;                           
                  } // switch (edit_state)     
                  break;
               case key_previous_parm :             //---------- PREV PARM key ---------- 
                  switch (edit_state) {
                     case edit_leg_mode:
                       if (current_edit_parm==0) current_edit_parm=3;
                         else  current_edit_parm--;
                       change_edit_leg_parm();                        
                       break;
                    case edit_offset_mode:
                       if (current_edit_parm==0) current_edit_parm=3;
                         else  current_edit_parm--;
                       change_edit_offset_parm();                  
                       break;
                    case edit_IK_mode:
                       if (current_edit_parm==0) current_edit_parm=3;
                         else  current_edit_parm--;
                       change_edit_IK_parm();                     
                       break;      
                    case run_mode:
                       if (current_edit_parm==0) current_edit_parm=1;
                         else  current_edit_parm--;
                       change_edit_run_parm();                     
                       break;    
                    case sensors_mode:
                       break;    
                  } // switch (edit_state) 
                  break;        
                case key_previous_page :              //---------- PREV PAGE key ---------- 
                  change_page=-1;  
                  break;       
                case key_next_page :                  //---------- NEXT PAGE key ---------- 
                  change_page=1;  
                  break;                           
               default :
                  break;
             } // switch(x)
        
             while(read_keyboard()==x0) ; 
             delay(delay_key);
             
         } // if (read_keyboard()==x0)   
         
     } // if (x>0)      
  
     if (digitalRead(clearButton) == LOW ) {            //---------- Encoder button ----------
       switch (edit_state) {
         case edit_leg_mode:
            break;
         case edit_offset_mode:
            break;
         case edit_IK_mode:    
            break;      
         case run_mode:
            break;    
         case sensors_mode:
            break;    
        }     
        while (digitalRead(clearButton) == LOW);
     }  // if

  if (change_page!=0) {
      update_edit_mode(change_page);     
      init_mode();
  }
}
