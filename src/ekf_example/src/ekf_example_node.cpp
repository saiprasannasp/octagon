#include <ros/ros.h>
#include <gps_example/GpsConversion.h>
#include <ekf_example/UnicycleEkf.h>
#include <nav_msgs/Odometry.h>

// Headers to publish TF frame
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

// Message headers
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>


ekf_example::UnicycleEkf* ekf;
gps_example::GpsConversion* gps;
ros::Publisher pub_path;
ros::Publisher pub_odom;
nav_msgs::Path path_msg;

double r_gps;
double r_v;
double r_gyro;
bool added_twist_measurements;

std::string parent_frame;
std::string child_frame;

//void recvTwist(const geometry_msgs::TwistStamped::ConstPtr& msg)
void recvTwist(const nav_msgs::Odometry::ConstPtr& msg)
{
  // Add forward speed and yaw rate measurements to Kalman filter
  if (!added_twist_measurements) { // Don't add measurement if one is already there
    added_twist_measurements = true;
    ekf->addVelMeas(msg->twist.twist.linear.x, r_v);
    ekf->addYawRateMeas(msg->twist.twist.angular.z, r_gyro);
  }
}

void recvGpsFix(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  // Convert current GPS measurement to ENU
  gps->setCurrentPosition(msg->latitude, msg->longitude, 284);

  // Extract converted ENU coordinates
  double enu_x;
  double enu_y;
  double enu_z;
  gps->getEnu(enu_x, enu_y, enu_z);

  ROS_INFO("(%f, %f, %f)", enu_x, enu_y, enu_z);


  // Add GPS measurement to current Kalman filter measurement
  ekf->addGpsMeas(enu_x, enu_y, r_gps);
}

void timerCallback(const ros::TimerEvent& event)
{
  // Advance Kalman filter one step and extract the current state estimate
  ekf->stepFilter();
  ekf_example::UnicycleState state = ekf->getState();
  added_twist_measurements = false; // Allow adding a new twist measurement

  // Update path and publish
  geometry_msgs::PoseStamped path_point;
  path_point.pose.position.x = state.x;
  path_point.pose.position.y = state.y;

  path_msg.header.frame_id = parent_frame;
  path_msg.header.stamp = event.current_real;
  path_msg.poses.push_back(path_point);
  pub_path.publish(path_msg);

  // Update TF transform
  static tf::TransformBroadcaster broadcaster;
  geometry_msgs::TransformStamped ekf_to_global;
  ekf_to_global.header.frame_id = parent_frame;
  ekf_to_global.header.stamp = event.current_real;
  ekf_to_global.child_frame_id = child_frame;
  ekf_to_global.transform.translation.x = state.x;
  ekf_to_global.transform.translation.y = state.y;
  ekf_to_global.transform.rotation = tf::createQuaternionMsgFromYaw(state.psi);
  broadcaster.sendTransform(ekf_to_global);

  // Populate odometry message
  nav_msgs::Odometry odom_msg;
  odom_msg.header = ekf_to_global.header;
  odom_msg.child_frame_id = ekf_to_global.child_frame_id;
  odom_msg.pose.pose.position.x = state.x;
  odom_msg.pose.pose.position.y = state.y;
  odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(state.psi);
  odom_msg.twist.twist.linear.x = state.v;
  odom_msg.twist.twist.angular.z = state.pdot;
  pub_odom.publish(odom_msg);

  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ekf_example_node");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

 
  // Load reference GPS points from parameters
  double ref_lat;
  double ref_lon;
  double ref_alt;
  n.param("ref_latitude", ref_lat, 42.672425);
  n.param("ref_longitude", ref_lon, -83.215259);
  n.param("ref_altitude", ref_alt, 284.0);

  ROS_INFO("%f, %f, %f", ref_lat, ref_lon, ref_alt);

  pn.param("parent_frame", parent_frame, std::string("/odom"));
  pn.param("child_frame", child_frame, std::string("/base_link"));

  gps = new gps_example::GpsConversion;
  gps->setRefCoordinates(ref_lat, ref_lon, ref_alt);

  // Initialize Kalman filter with state covariance values
  double q_pos;
  double q_psi;
  double q_v;
  double q_pdot;
  pn.param("q_pos", q_pos, 1.0);
  pn.param("q_psi", q_psi, 1.0);
  pn.param("q_v", q_v, 1.0);
  pn.param("q_pdot", q_pdot, 1.0);

  std::vector<double> qvals;
  qvals.resize(5);
  qvals[0] = q_pos; // x
  qvals[1] = q_pos; // y
  qvals[2] = q_psi; // psi
  qvals[3] = q_v; // v
  qvals[4] = q_pdot; // pdot
  ekf = new ekf_example::UnicycleEkf(qvals, 0.02);

  // Load measurement covariances from parameters
  pn.param("r_gps", r_gps, 1.0);
  pn.param("r_v", r_v, 1.0);
  pn.param("r_gyro", r_gyro, 1.0);

  // ROS Topics
  pub_path = n.advertise<nav_msgs::Path>("/ekf_path", 1);
  ros::Subscriber sub_gps_fix = n.subscribe("/fix", 1, recvGpsFix);
  ros::Subscriber sub_twist = n.subscribe("/odom_serial", 1, recvTwist);
  pub_odom = n.advertise<nav_msgs::Odometry>("/odom", 1);

  ros::Timer ekf_timer = n.createTimer(ros::Duration(0.02), timerCallback);

  ros::spin();

}
