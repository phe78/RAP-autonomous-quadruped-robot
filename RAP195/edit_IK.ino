//-----------------------------------------------------------------------------------
//   display IK values 
//-----------------------------------------------------------------------------------

void change_edit_IK_parm() {
       if (current_edit_parm==0) encoderPos = current_edit_leg;
       else encoderPos = leg_coor[current_edit_leg][current_edit_parm-1];
       lastReportedPos = encoderPos;   
       display_IK_values();    
       display_IK_values_active_parm();  
}
void display_IK_values() {
  int i;
  lcd.setCursor(1,1);
  lcd.print(current_edit_leg+1);
  
  for (i=0;i<3;i++) {
    lcd.setCursor(4+i*6,1);
    lcd.print("   ");
    lcd.setCursor(4+i*6,1);    
    lcd.print(leg_coor[current_edit_leg][i],0);
    }
}

void display_IK_values_update_parm(signed int n) {
    if (current_edit_parm==0) {
      if ((n>=0)&& (n<4)) current_edit_leg=n;
    }
    else {
      if ((current_edit_parm>0)&&(current_edit_parm<4)) {
          leg_coor[current_edit_leg][current_edit_parm-1]=n;
          move_IK_leg(current_edit_leg,default_speed);
      }
    }
    display_IK_values();    
}

void display_IK_values_active_parm() {
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

void display_IK_values_titles() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("IK");  
  lcd.setCursor(0,1);
  lcd.print("L");
  lcd.setCursor(3,1);
  lcd.print("X");
  lcd.setCursor(9,1);
  lcd.print("Y");
  lcd.setCursor(15,1);
  lcd.print("Z");
  lcd.setCursor(0,3);
  lcd.print("P-  P+  <   >   CLR");  
}

void clear_IK_values() {
  int i,j;
  for (i=0;i<leg_max;i++) 
     for (j=0;j<3;j++) {
        if (j==2) leg_coor[i][j]=default_heigth; 
           else leg_coor[i][j]=0;    
     }
  encoderPos=leg_coor[current_edit_leg][current_edit_parm-1];
  lastReportedPos = encoderPos;
}
