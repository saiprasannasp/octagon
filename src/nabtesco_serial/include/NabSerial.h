#ifndef __NABSERIAL__
#define __NABSERIAL__

#include <string>
namespace nabtesco {
	class NabSerial {

		std::string m_portname;
		int m_fd;
		int m_baud;
		float m_velocity;

	protected:
			
		
	public:
		NabSerial(std::string portname, int baud=0);
		~NabSerial();

		bool Init();
		bool ReadSerial();
		bool WriteSerial(char& buffer, int len);
		bool WriteSerial(float send);

		const float& GetVelocity();

		static void* ReadThread(void* nabSerial);
	};
}

#endif