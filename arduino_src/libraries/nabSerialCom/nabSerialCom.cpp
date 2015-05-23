#include "nabSerialCom.h"
#include "Arduino.h"


byte bkp_TCCR1B, bkp_TCCR0B, bkp_TCCR2B;

//Reading setpoint
boolean found = false;
byte count = 0;
byte reset = 0;
char bufffloat[8];
float test = 0.0;
extern float setpoint;

void readSetpoint(byte startr)
{  
    if(startr == 's')
    {
      //Serial.write(&startr, 1);
      found = false;
      count = 0;
      memset(bufffloat, '\0', 8);
      return;
    }
   
    if(count==0 || count==1 || count==6 || count==7) {
      if(startr == ':') {
        //Serial.write(&startr, 1);
        count++;
      } else {
        count=0;
      }
      
       if(count == 8)
       {
         memcpy(&test, &bufffloat, 4);
         setpoint = test;
         //Serial.write((uint8_t*)&test, 4);
         found = true;
         count = 0;
       }
      
       return;
    }
   
    if(count>=2 && count<6) {
      bufffloat[count-2] = startr;
      //Serial.write((uint8_t*)&bufffloat[count-2], 1);
      count++;
      return;
    }
   
    found = false;
}

void writeRPM(float* rpm)
{
  Serial.write((uint8_t*)"s::", 3);
  Serial.write((uint8_t*)rpm,4);
  Serial.write((uint8_t*)"::", 2);  
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

nabSerialCom::nabSerialCom() : m_available(false), m_val(0.0)
{
	Serial.begin(9600);
}

nabSerialCom::~nabSerialCom()
{

}

bool nabSerialCom::isAvailable()
{
	return m_available;
}

float nabSerialCom::getFloat()
{
	if(m_available) {
		m_available = false;
		return m_val;	
	}
}

bool nabSerialCom::read()
{
  if(Serial.available()>0)
  {  
    byte startr = Serial.read();
    
    if(startr == 's')
    {
      //Serial.write(&startr, 1);
      m_available = false;
      m_count = 0;
      memset(m_bufffloat, '\0', 8);
      return true;
    }
    
    if(m_count==0 || m_count==1 || m_count==6 || m_count==7) {
      if(startr == ':') {
        //Serial.write(&startr, 1);
        m_count++;
      } else {
        m_count=0;
      }
       
       if(m_count == 8)
       {
         memcpy(&m_val, &m_bufffloat, 4);
         //Serial.write((uint8_t*)&test, 4);
         m_available = true;
         m_count = 0;
         return true;
       }
       
       return false;
    }
    
    if(m_count>=2 && m_count<6) {
      m_bufffloat[m_count-2] = startr;
      //Serial.write((uint8_t*)&bufffloat[count-2], 1);
      m_count++;
      return false;
    }
    
    m_available = false;
  }

  return false;
}


