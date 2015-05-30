#include "myPID.h"
#include "Arduino.h"

/* we need to initilize Iterm to mid point of PWM to avoid kickbak at the start*/
float ITerm = 127.0, lastInput=0.0;
unsigned long lastTime, SampleTime=100;
float *myInput, *myOutput, *mySetpoint;
float kp=0.2, ki=0.4, kd=0.01;
float outMax=240, outMin=10;

/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/ 
bool PID_Compute()
{
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      float input = *myInput;
      float error = *mySetpoint - input;
      
      ITerm += (ki * error);
      if(ITerm > outMax) ITerm = outMax;
      else if(ITerm < outMin) ITerm= outMin;

      //if(ITerm > outMax) ITerm = outMax;
      //else if(ITerm < outMin) ITerm= outMin;

      float dInput = (input - lastInput);
      
      /*Compute PID Output*/
      
      float output = (kp * error) + ITerm-(kd * dInput);
      
      //Serial.print(ITerm);
      //Serial.print("::");
      //Serial.print(kp*error);
      //Serial.print("::");
      //Serial.println(rpm);
      //temp = temp+ITerm;
      //temp = output;

      if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;
      *myOutput = output;
	  
      /*Remember some variables for next time*/
      lastInput = input;
      lastTime = now;
      return true;
   } else { 
     return false;
   }
}