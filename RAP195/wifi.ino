//-----------------------------------------------------------------------------------
//   wifi routines
//-----------------------------------------------------------------------------------
void wifi_init() {
  sendToWifi("AT+CWMODE=2",responseTime,wifi_DEBUG);        // configure as access point
  sendToWifi("AT+CIFSR",responseTime,wifi_DEBUG);           // get ip address
  sendToWifi("AT+CIPMUX=1",responseTime,wifi_DEBUG);        // configure for multiple connections
  sendToWifi("AT+CIPSERVER=1,80",responseTime,wifi_DEBUG);  // turn on server on port 80
  sendToTeensy("Wifi connection is running!",responseTime,wifi_DEBUG);
}

/*
void loop()
{
  String message;
  if(Serial.available()>0){

     message = readSerialMessage();
     Serial.print("sending "); 
     Serial.println(message);         
     sendData(message);
    }

  if(wifiSerial.available()>0){
    message = readWifiSerialMessage();
    parse_received_message(message);
  }
  delay(responseTime);
}
*/

/*
* Name: sendData
* Description: Function used to send string to tcp client using cipsend
* Params: 
* Returns: void
*/
void sendData(String str){
  String len="";
  len+=str.length();
  sendToWifi("AT+CIPSEND=0,"+len,responseTime,wifi_DEBUG);
  delay(100);
  sendToWifi(str,responseTime,wifi_DEBUG);
  delay(100);
//  sendToWifi("AT+CIPCLOSE=5",responseTime,wifi_DEBUG);
}


/*
* Name: find
* Description: Function used to match two string
* Params: 
* Returns: true if match else false
*/
boolean find(String string, String value){
  if(string.indexOf(value)>=0)
    return true;
  return false;
}


/*
* Name: readSerialMessage
* Description: Function used to read data from Arduino Serial.
* Params: 
* Returns: The response from the Arduino (if there is a reponse)
*/
String  readSerialMessage(){
  char value[100]; 
  int index_count =0;
  while(Serial.available()>0){
    value[index_count]=Serial.read();
    index_count++;
    value[index_count] = '\0'; // Null terminate the string
  }
  String str(value);
  str.trim();
  return str;
}



/*
* Name: readWifiSerialMessage
* Description: Function used to read data from ESP8266 Serial.
* Params: 
* Returns: The response from the esp8266 (if there is a reponse)
*/
String  readWifiSerialMessage(){
  char value[100]; 
  int index_count =0;
  while(wifiSerial.available()>0){
    value[index_count]=wifiSerial.read();
    index_count++;
    value[index_count] = '\0'; // Null terminate the string
  }
  String str(value);
  str.trim();
  return str;
}



/*
* Name: sendToWifi
* Description: Function used to send data to ESP8266.
* Params: command - the data/command to send; timeout - the time to wait for a response; wifi_DEBUG - print to Serial window?(true = yes, false = no)
* Returns: The response from the esp8266 (if there is a reponse)
*/
String sendToWifi(String command, const int timeout, boolean wifi_DEBUG){
  String response = "",ipa;

  wifiSerial.println(command); // send the read character to the esp8266
  long int time = millis();
  while( (unsigned)(time+timeout) > millis())
  {
    while(wifiSerial.available())
    {
    // The esp has data so display its output to the serial window 
    char c = wifiSerial.read(); // read the next character.
    response+=c;
    }  
  }
  if(wifi_DEBUG)
  {
    Serial.println(response);    
    ipa=parse_IP_address(response);
    if (ipa!="") {
        IP_address=ipa;
        Serial.print("IP address : ");
        Serial.println(ipa);
    }
  }

  return response;
}

/*
* Name: sendToWifi
* Description: Function used to send data to ESP8266.
* Params: command - the data/command to send; timeout - the time to wait for a response; wifi_DEBUG - print to Serial window?(true = yes, false = no)
* Returns: The response from the esp8266 (if there is a reponse)
*/
String sendToTeensy(String command, const int timeout, boolean wifi_DEBUG){
  String response = "";
  Serial.println(command); // send the read character to the esp8266
  long int time = millis();
  while( (unsigned)(time+timeout) > millis())
  {
    while(Serial.available())
    {
      // The esp has data so display its output to the serial window 
      char c = Serial.read(); // read the next character.
      response+=c;
    }  
  }
  if(wifi_DEBUG)
  {
    Serial.println(response);
  }
  return response;
}

String parse_received_message(String message) {
  
int x,y;
char m[80];

  m[0]=0;
  if (find(message,"+IPD")) {
    message=message.substring(5,message.length());    
    const char *msg = message.c_str();
    sscanf(msg,"%d,%d:msg:%[^\n]s",&x,&y,m);
  }   
  return(m);
}

String parse_IP_address(String message) {

char m[80];

  m[0]=0;
  if (find(message,"+CIFS")) {
      message=message.substring(24,message.length()-20); 
      const char *msg = message.c_str(); 
      sscanf(msg,"%[^\"]s\n",m);        
  }   
  return(m);
}

