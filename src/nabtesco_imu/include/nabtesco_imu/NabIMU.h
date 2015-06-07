#ifndef __NAB_IMU_H__
#define __NAB_IMU_H__

#include <sstream>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include "ros/ros.h"

namespace octagon
{
	class NabIMU
	{
		ros::NodeHandle m_nh;
		std::string m_portname;
		int m_fd;
		int m_baud;
		float m_magx, m_magy, m_magz;
		float m_accx, m_accy, m_accz;
		pthread_mutex_t m_mutexsum;

		ros::Publisher m_magpub;

	public:
		NabIMU(ros::NodeHandle& n);
		~NabIMU();

		int Init();

		static void* ReadIMU(void* nabimu);
		void ReadSerial();
		void Update();
	};
} 

#endif