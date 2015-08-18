//nabtesco odometry class
#ifndef __NAB_ODOM__
#define __NAB_ODOM__

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/NavSatFix.h>


class NabOdom
{
	ros::Time m_current_time, m_last_time;
	double m_px, m_py, m_pth,gps_angle, odom_angle; //pose of the robot (x, y, theta)
        double lo_goal, la_goal,error_angle;
	float m_headingval;
	tf::TransformBroadcaster m_odom_broadcaster;
	ros::Publisher m_odom_pub;
	ros::Subscriber m_cmp_heading;
        ros::Subscriber m_gps;	
        ros::Subscriber m_goal_gps;	

public:
	NabOdom(ros::NodeHandle& n);
	~NabOdom();

	void Update(double vl_wheel, double vr_wheel);
	void Update(double vx, double vy, double vth);

	void rcvHeading(const std_msgs::Float32::ConstPtr& msg);
        void rcvGPS(const sensor_msgs::NavSatFix::ConstPtr& msg);
       void rcvGoalGPS(const geometry_msgs::PoseStamped::ConstPtr& msg);
};

#endif
