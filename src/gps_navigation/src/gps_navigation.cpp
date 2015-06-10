#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <gps_navigation/GpsConversion.h>


ros::Publisher pub_marker;
ros::Publisher pub_diff_cmd;
ros::Publisher pub_path;
ros::Publisher pub_goal_gps_msg;
visualization_msgs::Marker marker_msg;
visualization_msgs::MarkerArray marker_msg_array;
gps_navigation::GpsConversion* gps_points;
geometry_msgs::PoseStamped gps_point;
nav_msgs::Path path_msg;
geometry_msgs::TransformStamped transform_msg;
geometry_msgs::TwistStamped geom_msg;
geometry_msgs::PoseStamped goal_gps_msg;

double const_alt;
bool flag;
int counter;
double heading_angle;
double error_angle;

double Latitude[7];
double Longitude[7];

double x[7];
double y[7];
double z[7];

double la =0.0 , lo=0.0;
int ct = 0;

void recvGpsVel(const nav_msgs::Odometry::ConstPtr& msg) {
	double vx = msg->twist.twist.linear.x;
	double vy = msg->twist.twist.linear.y;

	/* Heading angle is computed with the 4-quadrant arctangent
	 of the velocity components */
	heading_angle = atan2(vy, vx);

	/* Converting from yaw angle to quaternion can be done with a TF function */
	transform_msg.transform.rotation = tf::createQuaternionMsgFromYaw(
			heading_angle);

}

void recvGps(const sensor_msgs::NavSatFix::ConstPtr& msg) {

	// Temporary point used to push into vector
	//geometry_msgs::PoseStamped gps_point;
	
	if(ct<10) {
		/*la = msg->latitude;
		lo =  msg->longitude;
		gps_points->setRefCoordinates(msg->latitude, msg->longitude, const_alt);*/
		ct++;
	}

	// Convert to ENU
	gps_points->setCurrentPosition(msg->latitude, msg->longitude, const_alt);

	// Extract ENU coordinates and place into temporary point
	gps_points->getEnu(gps_point.pose.position.x, gps_point.pose.position.y,
			gps_point.pose.position.z);

	// Push temporary point into vector
	path_msg.poses.push_back(gps_point);

	// compute the angle between the vehicle and the waypoint
	double sx = x[counter] - gps_point.pose.position.x;
	double sy = y[counter] - gps_point.pose.position.y;
	double psi = atan2(sy, sx);

	// compute the error angle
	error_angle = psi - heading_angle;
        ROS_INFO("Current: %f", gps_point.pose.position.x);
        ROS_INFO("GOAL: %f", x[counter]);
	// check if the vehicle hit the waypoint or within 0.5 meter radius
	if (sqrt(
			(x[counter] - gps_point.pose.position.x)
					* (x[counter] - gps_point.pose.position.x)
					+ (y[counter] - gps_point.pose.position.y)
							* (y[counter] - gps_point.pose.position.y))
			<= 0.5) {
                   ROS_INFO("BEFORE %d", counter);
		if (counter < 4) {
			counter++;
                        ct=0;
                        ROS_INFO("counter %d", counter);
			////////////////// To Move base/////////////////////////
			/*goal_gps_msg.pose.position.x = x[counter];
			goal_gps_msg.pose.position.y = y[counter];
			goal_gps_msg.pose.position.z = z[counter];
			goal_gps_msg.pose.orientation = transform_msg.transform.rotation;
	                goal_gps_msg.header.stamp = ros::Time().now();
			pub_goal_gps_msg.publish(goal_gps_msg);*/
			////////////////////////////////////////////////////////////////////
		} else if (counter >= 4) {
			geom_msg.twist.linear.x = 0.0;
			geom_msg.twist.angular.z = 0.0;
			flag = false;
		}
	}

	// Populate TF transform message with ENU coordinate points
	transform_msg.transform.translation.x = gps_point.pose.position.x;
	transform_msg.transform.translation.y = gps_point.pose.position.y;
	transform_msg.transform.translation.z = gps_point.pose.position.z;

}

/////////////////////////////////////////////////////
void timerCallback(const ros::TimerEvent& event) {

	static tf::TransformBroadcaster broadcaster;

	// Stamp the transform message and specify parent and child frames
	transform_msg.header.stamp = event.current_real;
	transform_msg.header.frame_id = "/odom";
	transform_msg.child_frame_id = "/base_link";
	//goal_gps_msg.header.stamp = event.current_real;


	// Send the transform to TF
	//broadcaster.sendTransform(transform_msg);

	// check if the vehicle reached the final point
	if (flag) {
		// populate the vehicle angle with the correct angle to head to the waypoint
		geom_msg.twist.angular.z = error_angle * 1.8;
	} else {
		geom_msg.twist.angular.z = 0.0;
	}

// populate and stamp msgs and publishers
	geom_msg.header.stamp = event.current_real;

	//pub_diff_cmd.publish(geom_msg);
	pub_marker.publish(marker_msg_array);
	path_msg.header.stamp = event.current_real;
	//pub_path.publish(path_msg);



        //goal_gps_msg.header.frame_id = "/odom";
	//goal_gps_msg.header.stamp = event.current_real;
	//goal_gps_msg.pose.position.x = x[counter];
	//goal_gps_msg.pose.position.y = y[counter];
	//goal_gps_msg.pose.position.z = z[counter];
	//goal_gps_msg.pose.orientation = transform_msg.transform.rotation;
	//pub_goal_gps_msg.publish(goal_gps_msg);

}
int main(int argc, char** argv) {
	ros::init(argc, argv, "navigation_node");
	ros::NodeHandle node;
	const_alt = 248.0;
	counter = 0;
	flag = true;

	pub_marker = node.advertise<visualization_msgs::MarkerArray>(
			"visualization_marker_array", 10);
	pub_diff_cmd = node.advertise<geometry_msgs::TwistStamped>(
			"/diff_steer/cmd", 1);
	//Buplishing the Goal to Move_base
		pub_goal_gps_msg = node.advertise<geometry_msgs::PoseStamped>(
				"/move_base_simple/goal", 1);

	ros::Subscriber sub_gps_fix = node.subscribe("/fix", 1, recvGps);
	ros::Subscriber sub_gps_vel = node.subscribe("/odom", 1, recvGpsVel);
	pub_path = node.advertise<nav_msgs::Path>("/gps_path", 1);

	// Timer
	ros::Timer main_timer = node.createTimer(ros::Duration(0.05),
			timerCallback);

	// Instantiate GpsConversion class and set geodetic reference coordinates
	gps_points = new gps_navigation::GpsConversion;
	gps_points->setRefCoordinates(42.6791355, -83.1960063333, const_alt);

	//populate the array with waypoints
	Latitude[0] = 42.6792685;
        Latitude[1] = 42.6793468333;
	Latitude[2] = 42.6793468333;
	Latitude[3] = 42.6793445;
	Latitude[4] = 42.679321;
	Latitude[5] = 42.679321;
	Latitude[6] = 42.679321;


        Longitude[0] = -83.196043;
	Longitude[1] = -83.195996;
	Longitude[2] = -83.195996;
	Longitude[3] = -83.1958468333;
	Longitude[4] = -83.1958303333;
	Longitude[5] = -83.1958303333;
	Longitude[6] = -83.1958303333;

	// Initialize marker constants for the waypoint
	marker_msg.header.frame_id = "/base_link";
	marker_msg.action = visualization_msgs::Marker::ADD;
	marker_msg.color.a = 0.9;
	marker_msg.color.g = 1;

	marker_msg.type = visualization_msgs::Marker::CYLINDER;
	marker_msg.scale.x = 1.0; // length
	marker_msg.scale.y = 1.0; // width
	marker_msg.scale.z = 0.2; // height

	// push the waypoints to the marker
	for (int i = 0; i < 7; i++) {

		gps_points->setCurrentPosition(Latitude[i], Longitude[i], const_alt);
		gps_points->getEnu(marker_msg.pose.position.x,
				marker_msg.pose.position.y, marker_msg.pose.position.z);
		x[i] = marker_msg.pose.position.x;
		y[i] = marker_msg.pose.position.y;
		z[i] = marker_msg.pose.position.z;
		marker_msg.header.stamp = ros::Time().now();
		marker_msg.id = i;
		marker_msg_array.markers.push_back(marker_msg);
	}

	




        
	
 	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		if(ct==10) {
			
			ct++;
			ROS_INFO("ct %d", ct);
			double x=0.0, y=0.0, z=0.0;
			gps_points->setCurrentPosition(la, lo, const_alt);
			gps_points->getEnu(x,y,z);		
			ROS_INFO("This is start X: %f, This is Y: %f" , x, y);
			gps_points->setCurrentPosition(Latitude[counter], Longitude[counter], const_alt);
			gps_points->getEnu(marker_msg.pose.position.x,
			marker_msg.pose.position.y, marker_msg.pose.position.z);
			x = marker_msg.pose.position.x;
			y = marker_msg.pose.position.y;
			z = marker_msg.pose.position.z;
			marker_msg.header.stamp = ros::Time().now();
			marker_msg.id = 1;
			//marker_msg_array.markers.push_back(marker_msg);

			goal_gps_msg.header.frame_id = "/odom";
			goal_gps_msg.header.stamp = ros::Time().now();
			goal_gps_msg.pose.position.x = x;
			goal_gps_msg.pose.position.y = y;
			goal_gps_msg.pose.position.z = 0.0;
			goal_gps_msg.pose.orientation = transform_msg.transform.rotation;
			pub_goal_gps_msg.publish(goal_gps_msg);
			ROS_INFO("This is goal X: %f, This is Y: %f" , goal_gps_msg.pose.position.x, goal_gps_msg.pose.position.y);

			// set the vehicle speed and stamp the frame id for msgs
			geom_msg.twist.linear.x = 1.4;
			path_msg.header.frame_id = "/odom";
			geom_msg.header.frame_id = "/odom";
		}

		ros::spinOnce();
 		loop_rate.sleep();
        }
}

