#include <NabSerial.h>
#include <sstream>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace nabtesco;
using namespace std;

NabSerial::NabSerial(string portname, int baud) :
	m_portname(""), m_fd(-1), m_baud(B9600)
{
  m_portname = portname;
  m_velocity = 0.0;
}

NabSerial::~NabSerial()
{

}

const float& NabSerial::GetVelocity() 
{ 
    //ROS_INFO("VEL %s %f", m_portname.c_str(), m_velocity);
    return m_velocity; 
}

bool NabSerial::ReadSerial()
{
	ROS_INFO("Inisde read thread");

  int rcvBufSize = 200;
  char ucResponse[8];//[rcvBufSize];   //response string from uController
  char *bufPos;
  std_msgs::String msg;
  std::stringstream ss;
  int BufPos,i;
  unsigned char crc_rx_sum =0;
  bool readflg = false;
  int count = 0;

  memset(ucResponse, '\0', 8);
  char bufffloat[4];

  ROS_INFO("Before entering loop");
  while (true) {
    //ROS_INFO("Found %f",m_velocity);
    char startr = '.';
    //ROS_INFO("fd %d"  , m_fd);
    ssize_t rd = read(m_fd, &startr, 1);
    //ssize_t rd = -1;       
    if(rd > 0) {

      //ROS_INFO("Bytes %c",startr);

      if(startr == 's') {
        count = 0;
        memset(bufffloat, '\0', 8);
        continue;
      }

      if(count == 0 || count == 1 || count == 6 || count == 7) {
        if( startr == ':')
          count++;
        else
          count = 0;

        if(count==8)
        {
          float test = 0.0;
          memcpy(&test, &bufffloat, sizeof(float));
          m_velocity = test;
          //ROS_INFO("Found %f",m_velocity);
        }
        continue;
      }

      if(count >= 2 && count <6) {
        bufffloat[count-2] = startr;
        count++;
        continue;
      }
    }
  }
}

bool NabSerial::WriteSerial(float send)
{
  //send++;
  char msg[4];

  float test1,test2;
  unsigned long i;

  memset(&msg, '\0', 4);
  memcpy((void*)&msg, (void*)&send, sizeof(float));

  char* ttf = (char*) &send;
  //ROS_INFO("sending float float %d %d %d %d", ttf[0], ttf[1], ttf[2], ttf[3]);
  //ROS_INFO("sending float str %d %d %d %d", msg[0], msg[1], msg[2], msg[3]);
  //ROS_INFO("sending float %f", send);
  //build the message packet to be sent
  char buff[9];
  memset(&buff, '\0', 9);
  buff[0] = 's'; buff[1] = ':'; buff[2] = ':';buff[7] = ':';buff[8] = ':';
  buff[3] = msg[0]; buff[4] = msg[1]; buff[5] = msg[2]; buff[6] = msg[3];
  //int n = sprintf(buff, "s::%s::", msg);
  //for (i=0;i<9;i++)
  //{
    //fprintf(g_fp, "%c", msg[i]);
  //}
  //char test = 's';  
  write(m_fd, &buff, 9);
  tcflush(m_fd, TCOFLUSH);
  //ROS_INFO("float sent %s %f", m_portname.c_str(), send);
  return true;
}

bool NabSerial::WriteSerial(char& buffer, int len)
{
  return false;
}

bool NabSerial::Init(void)
{
  ROS_INFO("%s", m_portname.c_str());
  m_fd = open(m_portname.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (m_fd == -1)
  {
    ROS_ERROR("open_port: Unable to open %s", m_portname.c_str());
    return false;
  }
  ROS_INFO("opening port %s successfull", m_portname.c_str());
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
  newtio.c_lflag = 0;

  // inter-charcter timer 
  newtio.c_cc[VTIME] = 0;

  // blocking read (blocks the read until the no.of charcters are read
  newtio.c_cc[VMIN] = 0;

  // clean the line and activate the settings for the port
  tcflush(m_fd, TCIFLUSH);
  tcsetattr (m_fd, TCSANOW,&newtio);

  //Open file as a standard I/O stream
  /*FILE* fp = fdopen(g_fd, "r+");

  if (!fp) {
      ROS_ERROR("serialInit: Failed to open serial stream /dev/ttyACM2");
      fp = NULL;
  }*/


  ROS_INFO("FileStandard I/O stream");
  pthread_t rcvThrID;   //receive thread ID
  int err;
  err = pthread_create(&rcvThrID, NULL, &NabSerial::ReadThread, this);
  if (err != 0) {
    ROS_ERROR("unable to create receive thread");
    return false;
  }

  return true;
}

void* NabSerial::ReadThread(void* nabSerial)
{
	((NabSerial*)nabSerial)->ReadSerial();
	return NULL;
}