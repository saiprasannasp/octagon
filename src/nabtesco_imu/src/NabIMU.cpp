#include <nabtesco_imu/NabIMU.h>
#include <vector>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <math.h>

using namespace octagon;

NabIMU::NabIMU(ros::NodeHandle& n):
m_nh(n), m_magx(0.0), m_magy(0.0), m_magz(0.0),
m_accx(0.0), m_accy(0.0), m_accz(0.0),  m_xoffset(-300)
{
	m_nh.param("/NabIMU/portname", m_portname, std::string("/dev/ttyUSB0"));
	m_nh.param("/NabIMU/baudrate", m_baud, B115200);

	m_magpub = n.advertise<geometry_msgs::Vector3>("magxyz", 1000);
	m_heading = n.advertise<std_msgs::Float32>("heading", 1000);
}

int NabIMU::Init()
{
	ROS_INFO("%s", m_portname.c_str());
	m_fd = open(m_portname.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if (m_fd == -1)
	{
		ROS_ERROR("open_port: Unable to open %s", m_portname.c_str());
		return false;
	}
	ROS_INFO("IMU opening port %s successfull", m_portname.c_str());

	struct termios newtio, oldtio;
	//else
	//fcntl(fd, F_SETFL, 0);

	// save current serial port settings
	tcgetattr(m_fd,&oldtio);
	// clear the struct for new port settings
	bzero(&newtio, sizeof(newtio));
	if (cfsetispeed(&newtio, m_baud) < 0 || cfsetospeed(&newtio, m_baud) < 0)
	{
		ROS_ERROR("serialInit: Failed to set serial baud rate: %d", m_baud);
		close(m_fd);
		return false;
	}

	ROS_INFO("copying old data successfull");
	long PARITYON = 0;
	long PARITY = 0;
	long STOPBITS = 0;	
	long DATABITS = CS8;

	// set baud rate, (8bit,noparity, 1 stopbit), local control, enable receiving characters.
	//newtio.c_cflag  = BAUD | CRTSCTS | CS8 | CLOCAL | CREAD;
	newtio.c_cflag = m_baud | DATABITS | STOPBITS | PARITYON | PARITY | CLOCAL | CREAD;
	// ignore bytes with parity errors
	newtio.c_iflag =  IGNPAR;
	// raw output
	newtio.c_oflag = 0;
	// set input mode to non - canonical
	newtio.c_lflag = ICANON;
	// inter-charcter timer 
	newtio.c_cc[VTIME] = 0;
	// blocking read (blocks the read until the no.of charcters are read
	newtio.c_cc[VMIN] = 1;
	// clean the line and activate the settings for the port
	tcflush(m_fd, TCIFLUSH);
	tcsetattr (m_fd, TCSANOW,&newtio);

	//cfsetispeed(&newtio, m_baud);
	//cfsetospeed(&newtio, m_baud); 
	fcntl(m_fd, F_SETFL, 0);

	pthread_mutex_init(&m_mutexsum, NULL);

	ROS_INFO("FileStandard I/O stream");
	pthread_t rcvThrID;   //receive thread ID
	int err;
	err = pthread_create(&rcvThrID, NULL, &NabIMU::ReadIMU, (void*)this);
	if (err != 0) {
		ROS_ERROR("unable to create receive thread");
		return false;
	}
}

NabIMU::~NabIMU()
{
	pthread_mutex_destroy(&m_mutexsum);
}

void NabIMU::ReadSerial()
{
	ROS_INFO("Inisde read thread");
	char* buff = (char*) malloc(sizeof(char)*2048);
	memset(buff, '\0', sizeof(char)*2048);
	int charcount = 0;

	ROS_INFO("Before entering loop");
	while(true)
	{
		
		char test='.';	
		int rd = read(m_fd, &test, sizeof(char));
		if(rd == -1 || test == '.')
		{
			ROS_INFO("Read Error %d", rd);
			continue;
		}
		if(test == 'T')
		{
			//ROS_INFO("before printing");
			charcount = 0;
			std::string str = buff;
			memset(buff, '\0', sizeof(char)*2048);
			//ROS_INFO("Found reading %d str %s", charcount, str.c_str());

			size_t pos = str.find("Mag:");
			if(pos == std::string::npos)
			{
				continue;
			}

			size_t apos = str.find("Accels:");
			std::string accstr = str.substr(apos+7);			
			std::stringstream accstrstream(accstr);
			std::vector<std::string> atokens;
			std::string aitem;
			//ROS_INFO("Mag info :");
			while (getline(accstrstream, aitem, '\t')) {
		        atokens.push_back(aitem);
		    	//ROS_INFO("%f", atof(item.c_str()));    
		    }

			std::string magstr = str.substr(pos+4);	
			
			std::stringstream magstrstream(magstr);
			std::vector<std::string> tokens;
			std::string item;
			//ROS_INFO("Mag info :");
			while (getline(magstrstream, item, '\t')) {
		        tokens.push_back(item);
		    	//ROS_INFO("%f", atof(item.c_str()));    
		    }

		    pthread_mutex_lock (&m_mutexsum);
		    m_magx = atof(tokens[0].c_str());
		    m_magy = atof(tokens[1].c_str());
		    m_magz = atof(tokens[2].c_str());
		    m_accx = atof(atokens[0].c_str());
		    m_accy = atof(atokens[1].c_str());
		    m_accz = atof(atokens[2].c_str()); 
		    pthread_mutex_unlock (&m_mutexsum);

		    //ROS_INFO("Mag Found reading %lu str %s", tokens.size(), magstr.c_str());
			//continue;
		}

		//ROS_INFO("setting char to buff %c", test);
		buff[charcount] = test;
		charcount++;
	}
}

void* NabIMU::ReadIMU(void* nabimu)
{
	((NabIMU*)nabimu)->ReadSerial();
}

void NabIMU::Update()
{
	geometry_msgs::Vector3 mag;
	mag.x=m_magx; mag.y=m_magy; mag.z=m_magz;

	float heading=0.0;
	/*if(m_magy>0) 
		heading = 90 - (atan2(m_magy,(m_magx-m_xoffset))*180/M_PI);
    	else if (m_magy<0) 
    		heading = 270 - (atan2(m_magy,(m_magx-m_xoffset))*180/M_PI);
	else if (m_magy==0 && m_magx<0) 
		heading = 180.0;
	else if(m_magy==0 && m_magx>0) 
		heading = 0.0;*/

	heading = atan2(m_magy,(m_magx-m_xoffset));
	ROS_INFO("Heading simple: %f", heading);

	//Magnetic declination: for ou igvc field -7° 27' 
	
	//Calculate the heading with tilt compentation.
	/*float accXNorm = m_accx/sqrt(m_accx*m_accx+m_accy*m_accy+m_accz*m_accz);
	float accYNorm = m_accy/sqrt(m_accx*m_accx+m_accy*m_accy+m_accz*m_accz);
	float pitch = asin(accXNorm);
    float roll = -asin(accYNorm/cos(pitch));

    float magXcomp = m_magx*cos(asin(accXNorm))+m_magz*sin(pitch);
    float magYcomp = m_magx*sin(asin(accYNorm/cos(pitch)))*sin(asin(accXNorm))+m_magy*cos(asin(accYNorm/cos(pitch)))-m_magz*sin(asin(accYNorm/cos(pitch)))*cos(asin(accXNorm));

    heading = 180*atan2(magYcomp,magXcomp)/M_PI;
    ROS_INFO("Heading tilt compensated: %f", heading);*/
	
	std_msgs::Float32 hdata;
	hdata.data = heading;
	m_heading.publish(hdata);
	m_magpub.publish(mag);
}
