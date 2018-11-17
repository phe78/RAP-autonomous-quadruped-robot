//-----------------------------------------------------------------------------------
//   encoder & keyboard routines
//-----------------------------------------------------------------------------------
void doEncoderA() {
  // debounce
  if ( rotating ) delay (1);  // wait a little until the bouncing is done

  // Test transition, did things really change?
  if ( digitalRead(encoderPinA) != A_set ) { // debounce once more
    A_set = !A_set;

    // adjust counter + if A leads B
    if ( A_set && !B_set )
      encoderPos += 1;

    rotating = false;  // no more debouncing until loop() hits again
  }
}
void doEncoderB() {
  if ( rotating ) delay (1);
  if ( digitalRead(encoderPinB) != B_set ) {
    B_set = !B_set;
    //  adjust counter - 1 if B leads A
    if ( B_set && !A_set )
      encoderPos -= 1;

    rotating = false;
  }
}

//-----------------------------------------------------------------------------------
//   keyboard
//-----------------------------------------------------------------------------------

int read_keyboard() {
  int k,i,d,t,n;
 k=analogRead(keyboardpin);

 d=2000;
 n=0;
 for (i=0; i<6; i++) {
  t=abs(k-keyboard_table[i]);
  if (t<d) {
    n=i+1;
    d=t;
  }
 }
 return(n-1);
}
//-----------------------------------------------------------------------------------
//   buzzer
//-----------------------------------------------------------------------------------

void long_tone() {
   digitalWrite(buzzerpin, HIGH);
   delay(200);
   digitalWrite(buzzerpin, LOW);  
}
void short_tone() {
   digitalWrite(buzzerpin, HIGH);
   delay(20);
   digitalWrite(buzzerpin, LOW);  
}
void tick_tone() {
   digitalWrite(buzzerpin, HIGH);
   delay(1);
   digitalWrite(buzzerpin, LOW);  
}
