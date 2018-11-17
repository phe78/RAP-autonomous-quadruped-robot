//-----------------------------------------------------------------------------------
//   utility routines
//-----------------------------------------------------------------------------------

void print_float(const char* s, float x,int d, boolean nl) {
  if (debug_mode) {
     Serial.print(s);
     Serial.print(x,d); 
     if (nl) Serial.println();
       else Serial.print(" ");
  }
}

void update_edit_mode(int n) {
  if (n==-1) {
      if (edit_state>edit_leg_mode) edit_state--;
        else edit_state = LIDAR_mode;
  } 
  if (n==1) {
      if (edit_state<LIDAR_mode) edit_state++;
         else edit_state = edit_leg_mode;
  }  
}

void init_mode() {

  change_page=0;
  current_edit_parm = 0;
  encoderPos = 0;  
  lastReportedPos = 0; 
  
  switch (edit_state) {
    case edit_leg_mode:
      display_leg_values_titles();
      display_leg_values_active_parm();
      display_leg_values();  
      break;
    case edit_offset_mode:
      display_offset_values_titles();
      display_offset_values_active_parm();
      display_offset_values();
      break;
    case edit_IK_mode:
      display_IK_values_titles();
      display_IK_values_active_parm();
      display_IK_values();
      break;      
    case run_mode:
      display_run_titles();
      display_run_active_parm();
      display_run_values();  
      change_edit_run_parm();    
      x=0;       
      break;    
    case sensors_mode:
      display_sensors_values_titles();
      display_sensors_values();
      break;    
    case LIDAR_mode:
      display_LIDAR_values_titles(); 
      break;      
  }
}


void splash_screen() {
  lcd.clear();
  lcd.setCursor(5,0);
  lcd.print("RAP V");    
  lcd.setCursor(10,0);
  lcd.print(rap_version);
  lcd.setCursor(4,1);
  lcd.print("Initializing");
  lcd.setCursor(1,3);
  lcd.print("Getting IP address");  
  delay(1000);
      
  Serial.print("RAP ");
  Serial.print(rap_version);
  Serial.println(" initializing...");
}

void display_IP_address(String ipa) {
  lcd.setCursor(1,3);
  lcd.print(" IP                 "); 
  lcd.setCursor(5,3);
  lcd.print(ipa);
}

void print_error(const char* err_name) {
  long_tone();
  Serial.println(err_name);
  lcd.setCursor(0,0);
  lcd.print("ERR ");
  lcd.setCursor(4,0);
  lcd.print(err_name); 
  while (digitalRead(clearButton) == HIGH);
  delay(20);
  while (digitalRead(clearButton) == LOW);
  lcd.setCursor(0,0);
  lcd.print("                    ");  
  lcd.setCursor(0,0);  
  switch (edit_state) {
    case edit_leg_mode:
      lcd.print("ANGLE");
      break;
    case edit_offset_mode:
      lcd.print("OFFSET");
      break;
    case edit_IK_mode:
      lcd.print("IK");
      break;      
    case run_mode:
      lcd.print("MOTION");
      break;    
    case sensors_mode:
      lcd.print("SENSORS");
      break;    
  }
}

void display_seqs() {
  int i,j,l,ls,t;
  Serial.println("**** dump seqs *****");

  
  for (i=0;i<max_seq;i++) {
    
    Serial.print("####### Sequ #");
    Serial.println(i);
    
    ls=last_step[i];
    if (ls==0) Serial.println("Sequence empty");
      else {
        Serial.print("Sequ steps : ");
        Serial.println(ls);
        
        for (j=0;j<ls;j++) {  
          Serial.print("Step#");
          Serial.println(j);        
          t=motion_seq[i][j][0][4];
              
          if (t==end_seq_tag) Serial.println("End");
            else if (t==loop_seq_tag) Serial.println("Loop");
            else if (t==delay_seq_tag) {
                Serial.print("Delay ");
                Serial.println(motion_seq[i][j][0][0]);
               }
            else if (t>=0) {
              Serial.println("Motion");
              for (l=0;l<leg_max;l++) {  
                  Serial.print("leg=");
                  Serial.print(l);      
                  Serial.print("  x=");
                  Serial.print(motion_seq[i][j][l][0]);
                  Serial.print("  y=");
                  Serial.print(motion_seq[i][j][l][1]);
                  Serial.print("  z=");
                  Serial.print(motion_seq[i][j][l][2]);  
                  Serial.print("  dz=");
                  Serial.print(motion_seq[i][j][l][3]);  
                  Serial.print("  dv=");
                  Serial.println(motion_seq[i][j][l][4]);                   
               } // for
            } // elseif           
          } // for
                  
        } //if
      } // for
}

void save_offset_values() {
  int i,j,a;
    a=EEPROM_offset_address;
    for (i=0 ; i<leg_max ; i++)
      for (j=0 ; j< leg_dof ; j++) {
        EEPROM.write(a,highByte(joint_offset[i][j]));
        a++;
        EEPROM.write(a,lowByte(joint_offset[i][j]));
        a++;
/*        Serial.print("save offset leg ");
        Serial.print(i);
        Serial.print(" joint ");
        Serial.print(j);
        Serial.print(" value ");       
        Serial.println(joint_offset[i][j]);  
*/     
      }
}

void clear_offset_values() {
  int i,j,a;
    a=EEPROM_offset_address;
    for (i=0 ; i<leg_max ; i++)
      for (j=0 ; j< leg_dof ; j++) {
        EEPROM.write(a,0);
        a++;
        EEPROM.write(a,0);
        a++;
        joint_offset[i][j]=0;   
      
      }
}
void load_offset_values() {
  int i,j,s;
  byte h,l;
    s=0;
    for (i=0 ; i<leg_max ; i++)
      for (j=0 ; j< leg_dof ; j++) {
        h=EEPROM.read(s+EEPROM_offset_address);
        s++;
        l=EEPROM.read(s+EEPROM_offset_address); 
        s++;      
        joint_offset[i][j]=(signed)word(h,l);
        if (joint_offset[i][j]>32767) joint_offset[i][j]=joint_offset[i][j]-65536;
/*        Serial.print("load offset leg ");
        Serial.print(i);
        Serial.print(" joint ");
        Serial.print(j);
        Serial.print(" value ");       
        Serial.println(joint_offset[i][j]);          
*/
      }    
}

