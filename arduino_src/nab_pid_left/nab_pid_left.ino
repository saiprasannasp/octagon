#define encoder0PinA 3
#define encoder0PinB 2
#define PWM_PIN 11
#define PWR_PIN 9
#define AUTO_ENABLE 5
#define ENCODER_TICK_COUT 100.0
#define GEAR_RATIO 100.0

//#include <PID_v1.h>
#include <nabSerialCom.h>
#include <myPID.h>

//Define Variables we'll be connecting to
float setpoint=0.0, rpm=0.0, pwm_output=143, prev_rpm=0.0, rpmA=0.0;
bool disabled = false;

extern byte bkp_TCCR1B, bkp_TCCR0B, bkp_TCCR2B;
extern unsigned long lastTime, SampleTime;
extern float *myInput, *myOutput, *mySetpoint;

//Specify the links and initial tuning parameters
//PID myPID(&rpm, &pwm_output, &setpoint, 3.7,0.8,0.05, DIRECT);

//stores encoder tick count!
volatile unsigned int encoder0Pos = 0;

//for increasing and decreasing speed.
int incomingByte = 0;

//rpm calc
int prev_deltaenc = 0; /*for roll over calc*/

//rpm = (delta_enc/deltatime) * (1000*60/500(encoder tick)*100(geartatio))
float pre_process_val = (float)(60000)/(float)(ENCODER_TICK_COUT*GEAR_RATIO);

//Debugging
unsigned int prev_encoder0Pos;
unsigned long lasttime, curtime;   
int counter=0, fltcount = 0;
boolean start= false, sc = false;

void setup()
{
  myInput = &rpm; myOutput=&pwm_output; mySetpoint=&setpoint;
  //pinMode(encoder0PinA, INPUT);
  // arduino mega 2560 
  // arduino ::  atmega2560 
  // pin 2   :: PortE pin4
  // pin 3   :: PortE pin5
  // arduino uno
  // arduino :: atmega 328
  // pin 2   :: PortD pin2
  // pin 3   :: PortD pin3
  
  // set up pins for encoder
  //this will set the pin to read or write mode
  //DDRE = 0b11100111; atmega 2560
  DDRD = 0b11110011;
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  
  pinMode(encoder0PinB, INPUT); 
  pinMode(encoder0PinA, INPUT);
  
  // This interrupt gives us encoder info
  attachInterrupt(0, doEncoder, RISING);  // encoder pin on interrupt 0 - pin 2

  pinMode(AUTO_ENABLE, INPUT);
  
  //set default value for PWM
  pinMode(PWM_PIN, OUTPUT);
  //setPwmFrequency(9, 256); // adjusts the frequency of the pwm sent to the b-bridge
  setPwmFrequency(PWM_PIN, 128);
  
  analogWrite(PWM_PIN, pwm_output);

  //For debugging 
  Serial.begin (9600);
  
  //init
  encoder0Pos=0;
  rpm = 0.0;
  lasttime=curtime=millis();
  //myPID.SetMode(AUTOMATIC);
  //myPID.SetOutputLimits(127.0, 240.0);
  lastTime = millis()-SampleTime;
}

void loop()
{
  //setpoint = 2.0;
  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    
    //readSetpoint(incomingByte);
    
    //'p' will increace the pwm val and 'l' will decrease the pwm val 
    /*if (incomingByte == 112)
      setpoint++;
    else if ( incomingByte == 108)
      setpoint--;
      */
  }
  
  //Debugging
  {
    unsigned int temp_encoder_val = encoder0Pos; 
    curtime = millis();
    unsigned int deltatime = curtime-lasttime;
    lasttime = curtime;
    
    int delta_enc = (temp_encoder_val-prev_encoder0Pos)*-1;
    /*deal with rollover*/
    if (temp_encoder_val == 65535 || temp_encoder_val == 0)
    {
      delta_enc = prev_deltaenc;
    }

    /*
      rpm = delta_enc*1000(ms)*60(s)/(ENCODER_TICK_COUT*deltatime*gearratio)
      in our case the above becomes
      rpm = (delta_enc/deltatime) * (1000*60/500(encoder tick)*100(geartatio))
    */
    if(deltatime == 0) {
      rpm = prev_rpm;
    } else {
      rpm = ((float)((float)delta_enc*pre_process_val)/(float)deltatime);
      prev_rpm=rpm;
    }
     
    prev_encoder0Pos = temp_encoder_val;
    prev_deltaenc = delta_enc;
    
    writeRPM(&rpm);
    //Serial.println(rpm);
    
    /*enable this to connect to radio*/
    if (digitalRead(AUTO_ENABLE) == LOW)
    {
      // say what you got:
      //Serial.println("OFF");
      disabled = true;
      TCCR1B = bkp_TCCR1B;
      TCCR1B = bkp_TCCR0B; 
      TCCR2B = bkp_TCCR2B;
      pinMode(PWM_PIN, INPUT);      
      delay(20);
      return;
    }
    
    if(disabled)
    {
      disabled = false;
      pinMode(PWM_PIN, OUTPUT);  
      setPwmFrequency(PWM_PIN, 128);
      pwm_output = 143.0;
      encoder0Pos=0;
      rpm=prev_rpm=setpoint=rpm=0.0;
      lasttime=curtime=millis();
      lastTime = millis()-SampleTime;
    }
    
    //myPID.Compute();
    PID_Compute();
    int temppwm = pwm_output;
    analogWrite(PWM_PIN, temppwm);
    
    //Serial.print(myPID.GetKi(), 4);
    //Serial.print("sai:");
    //Serial.print(myPID.GetKd(), 4);
    //Serial.println(rpm, 4);
    //Serial.print("  ");
    //Serial.print(setpoint, 4);
    //Serial.print("  ");
    //Serial.print(ITerm);
    //Serial.print("  ");
    //Serial.println(pwm_output);
    //Serial.println();
  }
  
  delay(10);
}

void doEncoder() {
    //if (PINE & (1<<PE5)) {  for atmega // check channel B to see which way wheel is turning
    if (PIND & (1<<PD3))   { //for uno 
      encoder0Pos--;         // CCW
    } else {
      encoder0Pos++;         // CW
    }
}
