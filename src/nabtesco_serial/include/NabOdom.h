//nabtesco odometry class
#ifndef __NAB_ODOM__
#define __NAB_ODOM__

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

class NabOdom
{
	ros::Time m_current_time, m_last_time;
	double m_px, m_py, m_pth; //pose of the robot (x, y, theta)
	tf::TransformBroadcaster m_odom_broadcaster;
	ros::Publisher m_odom_pub;

public:
	NabOdom(ros::NodeHandle& n);
	~NabOdom();

	void Update(double vl_wheel, double vr_wheel);
	void Update(double vx, double vy, double vth);
};

#endif