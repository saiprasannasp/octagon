#include <ros/ros.h>
#include <NabVision.h>

using namespace octagon;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "nabtesco_vision");
	ros::NodeHandle n;
	ROS_INFO("Nab vision node init");
	NabVision vision(n, n);
	ros::Rate loop_rate(30);

	while(ros::ok())
	{

		vision.Update();
		ros::spinOnce();
		loop_rate.sleep();	
	}
	
	return 0;
}