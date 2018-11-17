//-----------------------------------------------------------------------------------
//   display offset values 
//-----------------------------------------------------------------------------------

void change_edit_offset_parm() {
       if (current_edit_parm==0) encoderPos = current_edit_leg;
       else encoderPos = joint_offset[current_edit_leg][current_edit_parm-1];
       lastReportedPos = encoderPos;   
       display_offset_values();    
       display_offset_values_active_parm();  
}
void display_offset_values() {
  int i;
  lcd.setCursor(1,1);
  lcd.print(current_edit_leg+1);
  
  for (i=0;i<3;i++) {
    lcd.setCursor(4+i*6,1);
    lcd.print("   ");
    lcd.setCursor(4+i*6,1);    
    lcd.print(joint_offset[current_edit_leg][i]);
    move_servo_offset(current_edit_leg,i,joint_offset[current_edit_leg][i], default_speed); 
    }
}

void display_offset_values_update_parm(signed int n) {
    if (current_edit_parm==0) {
      if ((n>=0)&& (n<4)) current_edit_leg=n;
    }
    else {
      if ((current_edit_parm>0)&&(current_edit_parm<4)) {
        if (abs(n)<=offset_max) {
          joint_offset[current_edit_leg][current_edit_parm-1]=n;
          move_servo_offset(current_edit_leg,current_edit_parm-1,n, default_speed); 
        }
      }
    }
    display_offset_values();    
}

void display_offset_values_active_parm() {
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

void display_offset_values_titles() {
  lcd.clear();  
  lcd.setCursor(0,0);
  lcd.print("OFFSET");  
  lcd.setCursor(0,1);
  lcd.print("L");
  lcd.setCursor(3,1);
  lcd.print("C");
  lcd.setCursor(9,1);
  lcd.print("F");
  lcd.setCursor(15,1);
  lcd.print("T");
  lcd.setCursor(0,3);
  lcd.print("P-  P+  <   >   SET");  
}

