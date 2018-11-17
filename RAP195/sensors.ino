//-----------------------------------------------------------------------------------
//   display sensors values  (IMU, foot pressure sensors)
//-----------------------------------------------------------------------------------

void display_sensors_values() {
  int i;
  
    lcd.setCursor(2,1);    
    lcd.print("    ");
    lcd.setCursor(2,1);      
    lcd.print(int(ToDeg(pitch)));
    
    lcd.setCursor(9,1);    
    lcd.print("    ");
    lcd.setCursor(9,1);    
    lcd.print(int(ToDeg(roll)));  
      
    lcd.setCursor(16,1);    
    lcd.print("    ");
    lcd.setCursor(16,1);      
    lcd.print(int(ToDeg(yaw)));    

    for (i=0;i<4;i++) {
        lcd.setCursor(i*5+3,2);       
        if (leg_on_ground[i]) lcd.print("*");
            else lcd.print("-");                      
    }
}

void display_sensors_values_titles() {
  lcd.clear();  
  lcd.setCursor(0,0);
  lcd.print("SENSORS");
  lcd.setCursor(0,1);
  lcd.print("P");
  lcd.setCursor(7,1);
  lcd.print("R");
  lcd.setCursor(14,1);  
  lcd.print("Y");
  lcd.setCursor(0,2);  
  lcd.print("FL   FR   RL   RR  ");  
  lcd.setCursor(0,3);  
  lcd.print("P-  P+");  
}

void init_leg_sensors() {
  int i;
  
  for (i=0;i<4;i++) {
    leg_sensor[i]=analogRead(leg_sensor_pin[i]);
    leg_on_ground[i]=false;
  }
  
  leg_sensor_timer.begin(leg_sensor_interrupt, LEG_SENSOR_INTERRUPT_DELAY); 
}

void leg_sensor_interrupt() {
  xsens=analogRead(leg_sensor_pin[0]);
  leg_sensor[0]=xsens;
  if (xsens>leg_sensor_threshold[0]) leg_on_ground[0]=true;
     else leg_on_ground[0]=false;

  xsens=analogRead(leg_sensor_pin[1]);
  leg_sensor[1]=xsens;
  if (xsens>leg_sensor_threshold[1]) leg_on_ground[1]=true;
     else leg_on_ground[1]=false;

  xsens=analogRead(leg_sensor_pin[2]);
  leg_sensor[2]=xsens;
  if (xsens>leg_sensor_threshold[2]) leg_on_ground[2]=true;
     else leg_on_ground[2]=false;

  xsens=analogRead(leg_sensor_pin[3]);
  leg_sensor[3]=xsens;
  if (xsens>leg_sensor_threshold[3]) leg_on_ground[3]=true;
     else leg_on_ground[3]=false;
}

