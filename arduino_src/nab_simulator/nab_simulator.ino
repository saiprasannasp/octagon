#include <nabSerialCom.h>
#include <myPID.h>

//Define Variables we'll be connecting to
float setpoint=0.0, rpm=0.0;
bool disabled = false;

void setup()
{
   Serial.begin(9600);
}

void loop()
{
  //setpoint = 2.0;
  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    byte incomingByte = Serial.read();
    readSetpoint(incomingByte);
  }
  
  //simulate
  {
    rpm = setpoint;
    writeRPM(&rpm);
  }
  
  delay(10);
}

