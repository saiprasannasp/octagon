#include <nabtesco_imu/NabIMU.h>
#include "ros/ros.h"

using namespace octagon;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "NabIMUNode");
  	ros::NodeHandle n;
  	NabIMU _imu(n);
  	_imu.Init();

  	ros::Rate loop_rate(10);
  	while(ros::ok())
  	{
  		_imu.Update();
  		ros::spinOnce();
  		loop_rate.sleep();
  	}
	return 0;
}