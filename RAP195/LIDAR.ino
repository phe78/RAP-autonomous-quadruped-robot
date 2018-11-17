void LIDAR_init() {
    lidar.begin(LIDARSerial,128000); 
    pinMode(YDLIDAR_MOTOR_SCTP, OUTPUT);
    pinMode(YDLIDAR_MOTRO_EN, OUTPUT);
    LIDAR_on=false; 
}

void read_LIDAR() {

/*    if(isScanning){
        if(scans.count() > 0){
            scanPoint _point;
            _point = scans.pop();
            float distance = _point.distance; //distance value in mm unit
            float angle    =  _point.angle; //anglue value in degree
            if (distance>0) {
              LIDAR_dist[(int)angle]=distance;
              Serial.print("LIDAR_disti[");
              Serial.print((int)angle);
              Serial.print("]=");
              Serial.print(distance);
              Serial.println("]"); 
            }           
        }
    }else{
        //stop motor
  digitalWrite(YDLIDAR_MOTRO_EN, LOW);
        setMotorSpeed(0);
        restartScan();
    } */
      
    if(isScanning){
      if (lidar.waitScanDot() == RESULT_OK) {
          float distance = lidar.getCurrentScanPoint().distance; //distance value in mm unit
          float angle    = lidar.getCurrentScanPoint().angle; //anglue value in degree
            if (distance>0) {
              LIDAR_dist[(int)angle]=distance;
/*              Serial.print("LIDAR_disti[");
              Serial.print((int)angle);
              Serial.print("]=");
              Serial.print(distance);
              Serial.println("]"); */
            }           
      }else{
         Serial.println(" YDLIDAR get Scandata fialed!!");
      }
    }else{
        //stop motor
      digitalWrite(YDLIDAR_MOTRO_EN, LOW);
      setMotorSpeed(0);
      restartScan();
    }


}

void serialEvent() {
     if (lidar.waitScanDot() == RESULT_OK) {
          scanPoint _point = lidar.getCurrentScanPoint();
          if(scans.count() <= SIZE_OF_SCAN_BUFFER){
            scans.push(_point);
          }else{
            scans.pop();
            scans.push(_point);
          }
     }else{
       Serial.println(" YDLIDAR get Scandata fialed!!");
       //restartScan();
     }
}

void setMotorSpeed(float vol){
  uint8_t PWM = (uint8_t)(51*vol);
  analogWrite(YDLIDAR_MOTOR_SCTP, PWM);
}

void restartScan(){
    device_info deviceinfo;
    if (lidar.getDeviceInfo(deviceinfo, 100) == RESULT_OK) {

         int _samp_rate=4;
         String model;
         float freq = 7.0f;
         switch(deviceinfo.model){
            case 1:
                model="F4";
                _samp_rate=4;
                freq = 7.0;
                break;
            case 4:
                model="S4";
                _samp_rate=4;
                freq = 7.0;
                break;
            case 5:
                model="G4";
                _samp_rate=9;
                freq = 7.0;
                break;
            case 6:
                model="X4";
                _samp_rate=5;
                freq = 7.0;
                break;
            default:
                model = "Unknown";
          }

          uint16_t maxv = (uint16_t)(deviceinfo.firmware_version>>8);
          uint16_t midv = (uint16_t)(deviceinfo.firmware_version&0xff)/10;
          uint16_t minv = (uint16_t)(deviceinfo.firmware_version&0xff)%10;
          if(midv==0){
            midv = minv;
            minv = 0;
          }

          
          Serial.print("Firmware version:");
          Serial.print(maxv,DEC);
          Serial.print(".");
          Serial.print(midv,DEC);
          Serial.print(".");
          Serial.println(minv,DEC);

          Serial.print("Hardware version:");
          Serial.println((uint16_t)deviceinfo.hardware_version,DEC);

          Serial.print("Model:");
          Serial.println(model);

          Serial.print("Serial:");
          for (int i=0;i<16;i++){
            Serial.print(deviceinfo.serialnum[i]&0xff, DEC);
          }
          Serial.println("");

          Serial.print("[YDLIDAR INFO] Current Sampling Rate:");
          Serial.print(_samp_rate,DEC);
          Serial.println("K");

          Serial.print("[YDLIDAR INFO] Current Scan Frequency:");
          Serial.print(freq,DEC);
          Serial.println("Hz");
          delay(100);
          device_health healthinfo;
          if (lidar.getHealth(healthinfo, 100) == RESULT_OK){
             // detected...
              Serial.print("[YDLIDAR INFO] YDLIDAR running correctly! The health status:");
              Serial.println( healthinfo.status==0?"well":"bad");
              if(lidar.startScan() == RESULT_OK){
                isScanning = true;
                //start motor in 1.8v
            setMotorSpeed(0);
            digitalWrite(YDLIDAR_MOTRO_EN, HIGH);
                Serial.println("Now YDLIDAR is scanning ......");
              //delay(1000);
              }else{
                  Serial.println("start YDLIDAR is failed!  Continue........");
              }
          }else{
              Serial.println("cannot retrieve YDLIDAR health");
          }
  
      
       }else{
             Serial.println("YDLIDAR get DeviceInfo Error!!!");
       }
}

void display_LIDAR_values_titles() {
  lcd.clear();  
  lcd.setCursor(0,0);
  lcd.print("LIDAR");
  lcd.setCursor(0,3);
  lcd.print("P-  P+  - DMP R/S");  
}

void toggle_LIDAR_status() {
  if (LIDAR_on) {
      digitalWrite(YDLIDAR_MOTRO_EN, LOW);
      LIDAR_on = false;
  }
  else 
  {
      digitalWrite(YDLIDAR_MOTRO_EN, HIGH);
      LIDAR_on = true;    
  }
}  

void display_LIDAR_values() {

  unsigned int d,i,s;
  
  min_dist=100000;
  max_dist=0;
  min_dist_angle=0;
  max_dist_angle=0;
  s=0;  
  
  for (i=0;i<360;i++) {
    d=LIDAR_dist[i];
    if (d>0) {
      s++;
      if (d<min_dist) { 
        min_dist=d;
        min_dist_angle=i;
      }
      else if (d>max_dist) { 
        max_dist=d;
        max_dist_angle=i;
      }
    }
  }
  lcd.setCursor(10,0);
  lcd.print("  %");
  lcd.setCursor(10,0);  
  lcd.print((float)s/3.6f,0);
  lcd.setCursor(0,1);
  lcd.print("       ");
  lcd.setCursor(0,1);  
  lcd.print(min_dist_angle);     
  lcd.setCursor(10,1);
  lcd.print("       ");  
  lcd.setCursor(10,1);  
  lcd.print(min_dist);     
  lcd.setCursor(0,2);
  lcd.print("       ");  
  lcd.setCursor(0,2);  
  lcd.print(max_dist_angle);    
  lcd.setCursor(10,2);
  lcd.print("       ");
  lcd.setCursor(10,2);
  lcd.print(max_dist);    

}

void dump_LIDAR_dist() {
  for (int i=0;i<360;i++) {
    Serial.print(i);
    Serial.print(",");
    Serial.println(LIDAR_dist[i]);
  }
}

