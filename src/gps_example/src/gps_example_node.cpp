#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <gps_example/GpsConversion.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher pub_path;
ros::Publisher pub_gps_msg;
ros::Publisher pub_goal_gps_msg;
double const_alt;
double latitude;
double longitude;
bool flag = true;

gps_example::GpsConversion* gps;
nav_msgs::Path path_msg;
geometry_msgs::TransformStamped transform_msg;
geometry_msgs::PoseWithCovarianceStamped gps_msg;
geometry_msgs::PoseStamped goal_gps_msg;

/* This is the callback function that is called whenever
 a new GPS message is received */
void recvGps(const sensor_msgs::NavSatFix::ConstPtr& msg) {

	// Temporary point used to push into vector
	geometry_msgs::PoseStamped gps_point;

	// Convert to ENU
	gps->setCurrentPosition(msg->latitude, msg->longitude, const_alt);

	// Extract ENU coordinates and place into temporary point
	gps->getEnu(gps_point.pose.position.x, gps_point.pose.position.y,
			gps_point.pose.position.z);

	// Push temporary point into vector
	path_msg.poses.push_back(gps_point);

	// Populate TF transform message with ENU coordinate points
	transform_msg.transform.translation.x = gps_point.pose.position.x;
	transform_msg.transform.translation.y = gps_point.pose.position.y;
	transform_msg.transform.translation.z = gps_point.pose.position.z;

	//////////////////// FROM ROBOT GPS CURRENT //////////////////////
	gps_msg.pose.pose.position.x = gps_point.pose.position.x;
	gps_msg.pose.pose.position.y = gps_point.pose.position.y;
	gps_msg.pose.pose.position.z = gps_point.pose.position.z;

	/////////////////// END ROBOT GPS //////////////////////////////
}

/* This is the callback function that is called whenever
 the GPS velocity (east and north velocity) is received */
void recvGpsVel(const nav_msgs::Odometry::ConstPtr& msg) {
	double vx = msg->twist.twist.linear.x;
	double vy = msg->twist.twist.linear.y;

	/* Heading angle is computed with the 4-quadrant arctangent
	 of the velocity components */
	double psi = atan2(vy, vx);

	/* Converting from yaw angle to quaternion can be done with a TF function */
//  transform_msg.transform.rotation.w = cos(psi / 2);
//  transform_msg.transform.rotation.x = 0;
//  transform_msg.transform.rotation.y = 0;
//  transform_msg.transform.rotation.z = sin(psi / 2);
	transform_msg.transform.rotation = tf::createQuaternionMsgFromYaw(psi);
	gps_msg.pose.pose.orientation = transform_msg.transform.rotation;


}

/* Timer callback used to stamp path message and TF transform and publish */
void timerCallback(const ros::TimerEvent& event) {
	// Declare a static tf::TransformBroadcaster.  Shouldn't ever go out of scope
	static tf::TransformBroadcaster broadcaster;

	// Stamp the transform message and specify parent and child frames
	transform_msg.header.stamp = event.current_real;
	transform_msg.header.frame_id = "/map";
	transform_msg.child_frame_id = "/base_link";

	gps_msg.header.frame_id = "/map";
	gps_msg.header.stamp = event.current_real;

	// Send the transform to TF
	broadcaster.sendTransform(transform_msg);

	// Stamp and publish path message
	path_msg.header.stamp = event.current_real;
	pub_path.publish(path_msg);
	if (flag) {
		flag = false;
		pub_gps_msg.publish(gps_msg);
		// Convert to ENU
		gps->setCurrentPosition(latitude, longitude, const_alt);

		// Extract ENU coordinates and place into temporary point
		double x, y, z;
		gps->getEnu(x, y, z);
		goal_gps_msg.header.stamp = event.current_real;
		goal_gps_msg.header.frame_id = "/map";
		goal_gps_msg.pose.position.x = x;
		goal_gps_msg.pose.position.z = y;
		goal_gps_msg.pose.position.z = z;
		goal_gps_msg.pose.orientation = transform_msg.transform.rotation;
		pub_goal_gps_msg.publish(goal_gps_msg);

	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "gps_example_node");
	ros::NodeHandle node;
	latitude = 42.678842;
	longitude = -83.195283;
	const_alt = 284;
	ros::Subscriber sub_gps_fix = node.subscribe("/fix", 1, recvGps);
	ros::Subscriber sub_gps_vel = node.subscribe("/odom", 1, recvGpsVel);
	pub_path = node.advertise<nav_msgs::Path>("/gps_path", 1);
	pub_gps_msg = node.advertise<geometry_msgs::PoseWithCovarianceStamped>(
			"/initialpose", 1);
	pub_goal_gps_msg = node.advertise<geometry_msgs::PoseStamped>(
			"/move_base_simple/goal", 1);

	ros::Timer timer = node.createTimer(ros::Duration(0.05), timerCallback);

	// Instantiate GpsConversion class and set geodetic reference coordinates
	gps = new gps_example::GpsConversion;

	gps->setRefCoordinates(42.67264, -83.215039, const_alt);

	// Set constant frame id for the path message.
	path_msg.header.frame_id = "/map";

	ros::spin();
}
