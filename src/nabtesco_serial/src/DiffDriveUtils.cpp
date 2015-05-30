#include <DiffDriveUtils.h>
#include <ros/ros.h>
#include <ros/console.h>

# define M_PI  3.14159265358979323846  /* pi */

float DiffDriveUtils::m_wheel_raduis=0.0;
float DiffDriveUtils::m_wheel_base=0.0;

void DiffDriveUtils::convertRPMToVelocity(const float& rpm, float& res)
{
	//All units should be in meters and seconds so fo velocity we have m/s
	//since meters/minute calculation states that rpm*m_wheel_radius*2*M_PI
	//for meters/seconds calcumation cab be done as rpm*m_wheel_raduis*2*M_PI/60
	//grouping all the constants togeather we have 2*M_PI/60 = 0.10472
	res = rpm*DiffDriveUtils::m_wheel_raduis*0.10472;	
}

void DiffDriveUtils::convertTwistToVelocity(const geometry_msgs::Twist& twist, std_msgs::Float32& lvel, std_msgs::Float32& rvel)
{
	float dx  = twist.linear.x;
  	float dr = twist.angular.z;
  	rvel.data = -1.0*(dx+((dr * DiffDriveUtils::m_wheel_base)/2)); 
  	lvel.data = dx-((dr * DiffDriveUtils::m_wheel_base)/2);
}


void DiffDriveUtils::convertTwistToRPM(const geometry_msgs::Twist& twist, std_msgs::Float32& lvel, std_msgs::Float32& rvel)
{
	//First convert the twist to velocity.
	//then convert velocity to RPM
	//velocity to RPM calc =  vel/(0.10472*m_wheel_raduis)
	float dx  = twist.linear.x;
  	float dr = twist.angular.z;
  	rvel.data = (dx+((dr * DiffDriveUtils::m_wheel_base)/2))/(0.10472*DiffDriveUtils::m_wheel_raduis); 
  	lvel.data = (dx-((dr * DiffDriveUtils::m_wheel_base)/2))/(0.10472*DiffDriveUtils::m_wheel_raduis);

  	ROS_INFO("lwhel, rvel setpoint %f ,  %f", lvel.data, rvel.data);
}

void DiffDriveUtils::convertWheelVelToPose(const double& lvel, const double& rvel, double& xvel, double& yvel, double& thvel)
{
	thvel = (rvel-lvel)/DiffDriveUtils::m_wheel_base;
	xvel = ((rvel+lvel)/2)*cos(thvel);
	yvel = ((rvel+lvel)/2)*sin(thvel);
}