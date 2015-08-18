#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>

#define TS  0.05

ros::Publisher pub_integrated_path;
nav_msgs::Path integrated_path;
geometry_msgs::TransformStamped transform_msg;
geometry_msgs::PoseStamped pose_estimate;
geometry_msgs::Twist current_speed;

void recvVehicleSpeed(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  current_speed = msg->twist;
}

#define W   pose_estimate.pose.orientation.w
#define Z   pose_estimate.pose.orientation.z
void timerCallback(const ros::TimerEvent& event)
{
  // Static TF broadcaster
  static tf::TransformBroadcaster broadcaster;

  // Extract forward and steering speed from Twist message
  double vx = current_speed.linear.x;
  double psi_dot = current_speed.angular.z;

  // Compute the cosine and sine of the heading from quaternion:
  // cos(psi) = w^2 - z^2
  // sin(psi) = 2wz
  double cos_psi = W * W - Z * Z;
  double sin_psi = 2 * W * Z;

  // Actual angle can be computed from atan2
  double psi = atan2(sin_psi, cos_psi);

  /* Integrate current speed through state space model
   to increment estimate */
  pose_estimate.pose.position.x += TS * vx * cos_psi;
  pose_estimate.pose.position.y += TS * vx * sin_psi;
  psi += TS * psi_dot;

  // Update the pose estimate's orientation with the new value
  pose_estimate.pose.orientation = tf::createQuaternionMsgFromYaw(psi);

  // Push current estimated position into the path message
  integrated_path.poses.push_back(pose_estimate);

  // Stamp and publish path
  integrated_path.header.frame_id = "/map";
  integrated_path.header.stamp = event.current_real;
  pub_integrated_path.publish(integrated_path);

  // Populate a transform and send to TF
  geometry_msgs::TransformStamped vehicle_to_gps;
  vehicle_to_gps.header.frame_id = "/map";
  vehicle_to_gps.header.stamp = event.current_real;
  vehicle_to_gps.child_frame_id = "/robot_sensors"; /* A different frame than the GPS
                                                       node is publishing */
  vehicle_to_gps.transform.translation.x = pose_estimate.pose.position.x;
  vehicle_to_gps.transform.translation.y = pose_estimate.pose.position.y;
  vehicle_to_gps.transform.translation.z = pose_estimate.pose.position.z;
  vehicle_to_gps.transform.rotation.w = pose_estimate.pose.orientation.w;
  vehicle_to_gps.transform.rotation.x = pose_estimate.pose.orientation.x;
  vehicle_to_gps.transform.rotation.y = pose_estimate.pose.orientation.y;
  vehicle_to_gps.transform.rotation.z = pose_estimate.pose.orientation.z;
  broadcaster.sendTransform(vehicle_to_gps);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensor_integration");
  ros::NodeHandle node;

  pub_integrated_path = node.advertise<nav_msgs::Path>("/integrated_path", 1);
  ros::Subscriber sub_vehicle_speed = node.subscribe("/twist", 1, recvVehicleSpeed);

  ros::Timer timer = node.createTimer(ros::Duration(TS), timerCallback);

  ros::spin();
}
