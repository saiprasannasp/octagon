/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

//#include <PID_v1.h> // PID will incorporate encoder(input), pwm to motor(output), info. from computer(setpoint) 

#define encoder0PinA 2
#define encoder0PinB 3

//#define encodeIndesPin 3 //digital pin number 

// This code keeps track of an incremental number rEncoderPos.
// When the motor turns CW(looking at shaft end) it decrements
// CCW it increments

volatile unsigned int encoder0Pos = 0;
unsigned int previousPos = 0;

int radius = 4;
float encodercount = 500.0;

int rpm = 0;
int prev_encoder0Pos = 0;

//int rEncoderPinB = 4;
//int rEncoderPos = 0;
//volatile int indexCount = 0;

//int DIR = 12;
//int pwm = 0;// 26 = full reverse, 230 = full forward, 127 = stop
int pwmPin = 11;
//int pwmView = 12;
//int forward = 1;
//int inputRead;

/*float vHolder;
float velocity;
float disChange;
unsigned long timeOne = 0;
unsigned long timeTwo = 0;
unsigned long deltaTime = 0;
*/

//Define Variables we'll be connecting to
//double Setpoint, Input; 
double Output = 127;

//Specify the links and initial tuning parameters
//PID nabPID(&Input, &Output, &Setpoint,1,1,0, DIRECT);
int incomingByte = 0;

void setup()
{
 
 
  //pinMode(DIR, OUTPUT);    // Direction pin 
  
  // set up pins for right and left encoders as well as pins for right and left black bridge
  pinMode(encoder0PinA, INPUT); 
  digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor
  pinMode(encoder0PinB, INPUT); 
  digitalWrite(encoder0PinB, HIGH);       // turn on pullup resistor
  pinMode(pwmPin, OUTPUT);
  
  // This interrupt gives us encoder info
  attachInterrupt(0, doEncoder, HIGH);  // encoder pin on interrupt 0 - pin 2
  //attachInterrupt(1, doIndex, RISING);  // encoder pin on interrupt 0 - pin 3

  analogWrite(pwmPin,Output);
  //Setpoint = 20;
  setPwmFrequency(9, 256); // adjusts the frequency of the pwm sent to the b-bridge
  //nabPID.SetMode(AUTOMATIC); // turn PID on
  Serial.begin (9600);
  
  encoder0Pos=0;
  //indexCount=0;
  
  rpm = 0;
  prev_encoder0Pos = 0;
  //digitalWrite(DIR,HIGH);
}

void loop()
{
  
  
  //velocity;
  //nabPID.Compute();
  
  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();

    if (incomingByte == 112)
      Output++;
    else if ( incomingByte == 108)
      Output--;

    //Serial.println("pwmpin ::");
    analogWrite(pwmPin, Output);
  }
  
  /*{
    curtime = millis();
    int temp_encoder_val = encoder0Pos;
    
    
  }*/
  
  // Calculate change in time
  /*timeTwo = millis();
  deltaTime = timeTwo - timeOne;
  timeOne = timeTwo;
  
  // Keeps the value of disChange positive
  if(encoder0Pos > previousPos){
    disChange = encoder0Pos-previousPos;
    forward = 1;
  }
  else if(encoder0Pos == previousPos){
    forward = 0;
  }
  else{
    disChange = previousPos-encoder0Pos;
    forward = -1;
  }
  
  // Keeps the value of disChange steady when the numbers cycle from 64,000 to 0
  if(disChange > 1250){
   disChange = vHolder; 
  }
  vHolder = disChange;
  
  velocity = (float)(disChange)*(10)/(deltaTime*.1); // Velocity is in the form of rpm
  previousPos = (int)encoder0Pos;
  */
  
  //analogWrite(pwmPin,Output);
  //analogWrite(pwmView,Output);
  
  /*Serial.print( "   velocity: ");
  Serial.print( velocity );
  Serial.print("    ");
  Serial.print("Setpoint: ");
  Serial.print (Setpoint);
  Serial.print("   ");
  Serial.print("Output: ");
  Serial.print(Output);
  Serial.print("   forward: ");
  Serial.print(forward);
  Serial.print('\n');*/
  
  // say what you got:
  //Serial.print(Output, DEC);
  //Serial.print("  ");
  Serial.println(encoder0Pos, DEC);
 
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

/*void doIndex() {
  indexCount++;
}*/

void doEncoder() {
  /* If pinA and pinB are both high or both low, it is spinning
   * forward. If they're different, it's going backward.
   *
   * For more information on speeding up this process, see
   * [Reference/PortManipulation], specifically the PIND register.
   */
  if (digitalRead(encoder0PinA) == HIGH) {   // found a low-to-high on channel A
    if (digitalRead(encoder0PinB) == LOW) {  // check channel B to see which way                                         // encoder is turning
      encoder0Pos--;         // CCW
    } else {
      encoder0Pos++;         // CW
    }
  } /*else {
    if (digitalRead(encoder0PinB) == LOW) {   // check channel B to see which way
                                              // encoder is turning  
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }*/

  //Serial.println (encoder0Pos, DEC);
}

/* See this expanded function to get a better understanding of the
 * meanings of the four possible (pinA, pinB) value pairs:
 */
/*void doEncoder_Expanded(){
  if (digitalRead(encoder0PinA) == HIGH) {   // found a low-to-high on channel A
    if (digitalRead(encoder0PinB) == LOW) {  // check channel B to see which way
                                             // encoder is turning
      encoder0Pos = encoder0Pos - 1;         // CCW
    } 
    else {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
  }
  else                                        // found a high-to-low on channel A
  { 
    if (digitalRead(encoder0PinB) == LOW) {   // check channel B to see which way
                                              // encoder is turning  
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }

  }
  //Serial.println (encoder0Pos, DEC);          // debug - remember to comment out
                                              // before final program run
  // you don't want serial slowing down your program if not needed
}*/

