#ifndef __DIFFDRIVEUTILS__
#define __DIFFDRIVEUTILS__

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

class DiffDriveUtils
{
public:
	static float m_wheel_raduis;
	static float m_wheel_base;
	static void convertRPMToVelocity(const float& rpm, float& res);
	static void convertTwistToVelocity(const geometry_msgs::Twist& twist, std_msgs::Float32& lvel, std_msgs::Float32& rvel);
	static void convertTwistToRPM(const geometry_msgs::Twist& twist, std_msgs::Float32& lvel, std_msgs::Float32& rvel);
	static void convertWheelVelToPose(const double& lvel, const double& rvel, double& xvel, double& yvel, double& thvel);
};

#endif