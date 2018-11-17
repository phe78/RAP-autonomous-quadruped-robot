//-----------------------------------------------------------------------------------
//   display leg values 
//-----------------------------------------------------------------------------------

void change_edit_leg_parm() {
       if (current_edit_parm==0) encoderPos = current_edit_leg;
       else encoderPos = joint_angle[current_edit_leg][current_edit_parm-1];
       lastReportedPos = encoderPos;   
       display_leg_values();    
       display_leg_values_active_parm();  
}

void display_leg_values() {
  int i;
  lcd.setCursor(1,1);
  lcd.print(current_edit_leg+1);
  
  for (i=0;i<3;i++) {
    lcd.setCursor(4+i*6,1);
    lcd.print("   ");
    lcd.setCursor(4+i*6,1);    
    lcd.print(joint_angle[current_edit_leg][i]);
    }
}

void display_leg_values_update_parm(signed int n) {
    if (current_edit_parm==0) {
      if ((n>=0)&& (n<4)) current_edit_leg=n;
    }
    else {
      if ((current_edit_parm>0)&&(current_edit_parm<5)) {
        if (abs(n)<=servo_angle_max/2) {
          joint_angle[current_edit_leg][current_edit_parm-1]=n;
          move_servo(current_edit_leg, current_edit_parm-1, default_speed,terminate_command); 
        }
      }
    }
    display_leg_values();    
}

void display_leg_values_active_parm() {
  int i;

  if (current_edit_parm==0) {
    lcd.setCursor(0,2);
    lcd.print("*");
    for (i=0;i<3;i++) {
       lcd.setCursor(3+i*6,2);
       lcd.print(" ");
       } 
  } 
  else
  {
    lcd.setCursor(0,2);
    lcd.print(" ");
    for (i=0;i<3;i++) {
       if (i==current_edit_parm-1) {
         lcd.setCursor(3+i*6,2);
         lcd.print("*");
         }
       else {
         lcd.setCursor(3+i*6,2);
         lcd.print(" ");
         } // else
       } // for
    } // if
}

void display_leg_values_titles() {
  lcd.clear();  
  lcd.setCursor(0,0);
  lcd.print("ANGLE");
  lcd.setCursor(0,1);
  lcd.print("L");
  lcd.setCursor(3,1);
  lcd.print("C");
  lcd.setCursor(9,1);
  lcd.print("F");
  lcd.setCursor(15,1);
  lcd.print("T");
  lcd.setCursor(0,3);
  lcd.print("P-  P+  <   >  CLR");  
}

