//-----------------------------------------------------------------------------------
//   display IK values 
//-----------------------------------------------------------------------------------

void change_edit_run_parm() {

       switch (current_edit_parm) {
          case 0:
            encoderPos = body_heigth;         
            break;
          case 1:
            encoderPos = current_seq;
            break;
          default:
            break;
       }
       lastReportedPos = encoderPos;   
       display_run_values();    
       display_run_active_parm();  
}

void display_run_values() {
  
    lcd.setCursor(7,1);
    lcd.print("   ");
    lcd.setCursor(7,1);    
    lcd.print(body_heigth,0);
    lcd.setCursor(15,1);
    lcd.print("   ");
    lcd.setCursor(15,1);    
    lcd.print(current_seq);    
}

void display_run_values_update_parm(signed int n) {

       switch (current_edit_parm) {
          case 0:
            if (n<max_heigth) { 
              body_heigth=n;
              reset_all_legs(default_speed);  
            }
            break;
          case 1:
                  if ((n>=0)&& (n<max_seq)) current_seq=n;
            break;
          default:
            break;
       }  
    display_run_values();    
}

void display_run_active_parm() {
  int i;

    for (i=0;i<2;i++) {
       if (i==current_edit_parm) {
         lcd.setCursor(7+i*8,2);
         lcd.print("*");
         }
       else {
         lcd.setCursor(7+i*8,2);
         lcd.print(" ");
         } // else
     } // for
}

void display_run_titles() {
  
  lcd.clear();  
  lcd.setCursor(0,0);
  lcd.print("MOTION");  
  lcd.setCursor(0,1);
  lcd.print("Height");
  lcd.setCursor(11,1);
  lcd.print("Seq");
  lcd.setCursor(0,3);
  lcd.print("P-  P+  <   >   RUN");  
}

