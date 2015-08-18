#include <NabOdom.h>
#include <nav_msgs/Odometry.h>
#include <DiffDriveUtils.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>

NabOdom::NabOdom(ros::NodeHandle& n) : m_headingval(0.0)
{
	//initialize the pose of the robot to zero. means robot starting at origin of odom frame.
	m_odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	//m_cmp_heading = n.subscribe("heading", 1, &NabOdom::rcvHeading, this);
        //m_gps = n.subscribe("/fix", 1, &NabOdom::rcvGPS, this);
        //m_goal_gps = n.subscribe("/move_base_simple/goal", 1, &NabOdom::rcvGoalGPS, this);

	m_px=m_py=m_pth=0.0;

	//initialize time
  	m_current_time = ros::Time::now();
  	m_last_time = ros::Time::now();
}

NabOdom::~NabOdom()
{

}

void NabOdom::rcvGPS(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	double  la = msg->latitude;
        double lo =  msg->longitude;
        double sx = la_goal - la;
        double sy = lo_goal-lo;
        double gps_angle = atan2(sy, sx);
        error_angle =gps_angle;//= (gps_angle - m_headingval)* 0.9;
	
}

void NabOdom::rcvGoalGPS(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	la_goal = msg->pose.position.x;
        lo_goal =msg->pose.position.y;
}



void NabOdom::rcvHeading(const std_msgs::Float32::ConstPtr& msg)
{
	//ROS_INFO("I heard heading: [%f]", msg->data);
	m_headingval = msg->data;
	
}

void NabOdom::Update(double vl_wheel, double vr_wheel)
{
	double vx=0.0, vy=0.0, vth = 0.0;
	DiffDriveUtils::convertWheelVelToPose(vl_wheel, vr_wheel, vx, vy, vth);
	ROS_INFO("NAB_VEL update %f  %f", vl_wheel, vr_wheel);
	ROS_INFO("NAB_ODOM update %f  %f  %f", vx, vy, vth);
	Update(vx, vy, vth);
        
}

//input 1. velocity of wheel in x direction
//		2. velocity of wheel in y direction	
//		3. rate of yaw (orientation)
void NabOdom::Update(double vx, double vy, double vth)
{
	m_current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (m_current_time - m_last_time).toSec();
    double delta_x = (vx * cos(m_pth) - vy * sin(m_pth)) * dt;
    double delta_y = (vx * sin(m_pth) + vy * cos(m_pth)) * dt;
    double delta_th = vth * dt;

    m_px += delta_x;
    m_py += delta_y;
    m_pth += delta_th;
    //m_pth = m_headingval;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(m_pth);//error_angle);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = m_current_time;
    odom_trans.header.frame_id = "/odom";
    odom_trans.child_frame_id = "/base_link";

    odom_trans.transform.translation.x = m_px;
    odom_trans.transform.translation.y = m_py;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    m_odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = m_current_time;
    odom.header.frame_id = "/odom";

    //set the position
    odom.pose.pose.position.x = m_px;
    odom.pose.pose.position.y = m_py;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "/base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    m_odom_pub.publish(odom);

    m_last_time = m_current_time;
}
