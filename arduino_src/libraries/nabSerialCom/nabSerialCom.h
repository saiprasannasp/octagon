#ifndef __NABSERIALCOM__
#define __NABSERIALCOM__

#include "Arduino.h"

void setPwmFrequency(int pin, int divisor);
void readSetpoint(byte startr);
void writeRPM(float* rpm);

class nabSerialCom {

private:
	unsigned int m_count;
	char m_bufffloat[8];	
	bool m_available;
	
public:
	float m_val;

	nabSerialCom();
	~nabSerialCom();
	
	bool isAvailable();
	float getFloat();

	bool read();

};

#endif