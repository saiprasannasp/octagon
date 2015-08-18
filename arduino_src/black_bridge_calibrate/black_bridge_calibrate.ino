int pwmPin = 11;
byte bkp_TCCR1B, bkp_TCCR0B, bkp_TCCR2B;

void setup(){
 pinMode(pwmPin, OUTPUT); 
 
 //setPwmFrequency(9, 256); // adjusts the frequency of the pwm sent to the b-bridge
  setPwmFrequency(pwmPin, 128);
  
  //analogWrite(pwmPin, 137);
}



void loop(){
 analogWrite(pwmPin, 245);
 delay(10000);
 
 analogWrite(pwmPin, 10);
 delay(10000);
 
 analogWrite(pwmPin, 143);
 delay(10000);
}

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  bkp_TCCR1B = TCCR1B;
  bkp_TCCR0B = TCCR0B; 
  bkp_TCCR2B = TCCR2B;
   
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

