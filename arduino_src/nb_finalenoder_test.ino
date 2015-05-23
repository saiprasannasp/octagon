#define encoder0PinA 2
#define encoder0PinB 3
#define PWM_PIN 11

//stores encoder tick count!
volatile unsigned int encoder0Pos = 0;

//Pwm default value.
double pwm_output = 127;

//for increasing and decreasing speed.
int incomingByte = 0;

//Debugging
unsigned int prev_encoder0Pos;
unsigned long lasttime, curtime;

void setup()
{
 
  //pinMode(encoder0PinA, INPUT);
  // arduino ::  atmega2560 
  // pin 2   :: PortE pin4
  // pin 3   :: PortE pin5
  
  // set up pins for encoder
  //this will set the pin to read or write mode
  DDRE = 0b11100111;
  pinMode(encoder0PinB, INPUT); 
    
  // This interrupt gives us encoder info
  attachInterrupt(0, doEncoder, RISING);  // encoder pin on interrupt 0 - pin 2

  //set default value for PWM
  pinMode(PWM_PIN, OUTPUT);
  setPwmFrequency(9, 256); // adjusts the frequency of the pwm sent to the b-bridge
  analogWrite(PWM_PIN, pwm_output);

  //For debugging 
  Serial.begin (9600);
  
  //init
  encoder0Pos=0;
  lasttime=curtime=millis();
}

void loop()
{
  
  
  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    
    //'p' will increace the pwm val and 'l' will decrease the pwm val 
    if (incomingByte == 112)
      pwm_output++;
    else if ( incomingByte == 108)
      pwm_output--;
      
    analogWrite(PWM_PIN, pwm_output);
  }
  
  //Debugging
  {
    unsigned int temp_encoder_val = encoder0Pos; 
    curtime = millis();
    unsigned int deltatime = curtime-lasttime;
    lasttime = curtime;
    unsigned int delta_enc = temp_encoder_val-prev_encoder0Pos;
    prev_encoder0Pos = temp_encoder_val;
    
    // say what you got:
    Serial.print(deltatime, DEC);
    Serial.print("  ");
    Serial.print(delta_enc, DEC);
    Serial.print("  ");
    Serial.println(temp_encoder_val, DEC);
  }
}

void setPwmFrequency(int pin, int divisor) {
  byte mode;
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

void doEncoder() {
    if (PINE & (1<<PE5)) {  // check channel B to see which way                                         // encoder is turning
      encoder0Pos--;         // CCW
    } else {
      encoder0Pos++;         // CW
    }
}
