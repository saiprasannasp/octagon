/* 0-170 : 170 full forward, 0 back
 * 171-255 : 255 is full left, 171 is full right
 *
 * Forward Velocity: twist.linear.x
 * units: m/s	  max: +-1.35 m/s
 *
 * Angular Velocity: twist.angular.z
 * units: rad/s	  max: +-2.44 rad/s
 */

#define WHEEL_RADIUS	(0.1016)	// meters
#define WHEEL_SPACING	(0.552)	// meters

// Serial Port
#define BAUD_RATE B9600
#include <veh_speed/SerialPort.h>

// ROS communication
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>

// Publishers
ros::Publisher auto_pub;
ros::Publisher twist_pub;
ros::Publisher odom_pub;

#define LOOP_RATE	20.0	// 100Hz

// Globals
geometry_msgs::Twist g_twist_cmd;
geometry_msgs::TwistStamped veh_speed_twist;
geometry_msgs::Pose robot_pose;
nav_msgs::Odometry odom_msg;
bool g_autonomous = false;	// Autonomous mode enabled
SerialPort *g_port;
SerialPort *g_port2;
double vLeft;
double vRight;
double ts;
double left_angle;
double right_angle;
std::string left_port;
std::string right_port;

void publishAutonomous() {
std_msgs::Bool msg;
//msg.data = g_autonomous;
msg.data = true;
auto_pub.publish(msg);
}

#define SPEED_MAX	9.99;
#define SPEED_MIN	-9.9;
#define ROTATION_MAX	255
#define ROTATION_MIN	171

void Write(SerialPort *dev) {
double vf = g_twist_cmd.linear.x;
double vs = g_twist_cmd.angular.z;
if (vf > 9.99)
{
vf=9.99;
}
if (vf < -9.9)
{
vf = -9.9;
}

double x = vf / (WHEEL_RADIUS / 2.0);
double y = vs / (WHEEL_RADIUS / WHEEL_SPACING);
double wl = (x - y) / 2.0;
double wr = y + wl;

std::ostringstream strsRight;
std::ostringstream strsLeft;
strsRight << wr;
strsLeft << wl;
std::string sr = strsRight.str();
std::string sl = strsLeft.str();

char buf[4];
char *ptr = (char *) buf;

//ROS_INFO("Fwd: %d, Steer: %d", buf[0], buf[1]);

if (dev->portName == (char*) right_port.c_str()) {
// write to right wheel
buf[0] = sr[0];
buf[1] = sr[1];
buf[2] = sr[2];
buf[3] = sr[3];
//	ROS_INFO("Right: %d", buf[0]);
if (dev->writeData(ptr, 4) < 0) {
dev->closePort();
}
}
if (dev->portName == (char*) left_port.c_str()) {
// write to left wheel
buf[0] = sl[0];
buf[1] = sl[1];
buf[2] = sl[2];
buf[3] = sl[3];
//	ROS_INFO("Left: %d", buf[0]);
if (dev->writeData(ptr, 4) < 0) {
dev->closePort();
}
}

}
void Read(SerialPort *dev) {

unsigned char buffer[1024];
unsigned char *ptr = buffer;
int length = dev->readData(ptr, sizeof(buffer));
if (length >= 9) {
//	ROS_INFO("Leangth is: %d",length);
int i = 10;
while (i <= length) {
if (buffer[i] == '\n') {
break;
}
i++;

}
if (buffer[i] == '\n') {
std::string strBufer(reinterpret_cast<char const*>(buffer));
//	ROS_INFO_STREAM("the whole string: "+strBufer);
//	ROS_INFO("I is: %d",i);
double cnt_ms = 0.00;
if (strBufer[i - 2] == 'L') {
left_port = dev->portName;
std::string sLeft = strBufer.substr(i - 8, 6);
vLeft = atof(sLeft.c_str()) / 60 * 2 * M_PI;
return;
//	ROS_INFO("Left Before %f",cnt_ms);
//	vLeft = cnt_ms * 2.0 * M_PI * 4.0 * 0.0254 * (1.0 / 60.0);// m/s
//	if (vLeft >= -9.99 && vLeft <= 9.00) {
//	ROS_INFO("this is vLeft: %f", vLeft);
//	return;
//	} else {
//	vLeft = 0.00;
//	return;
//	}
}
if (strBufer[i - 2] == 'R') {
right_port = dev->portName;
std::string sRight = strBufer.substr(i - 8, 6);
vRight = atof(sRight.c_str()) / 60 * 2 * M_PI;
return;
////	ROS_INFO("Right Before %f",cnt_ms);
//	vRight = cnt_ms * 2.0 * M_PI * 4.0 * 0.0254 * (1.0 / 60.0);	// m/s
//	if (vRight >= -9.99 && vRight <= 9.00) {
////	ROS_INFO("this is vRight: %f", vRight);
//	return;
//	} else {
//	vRight = 0.00;
//	return;
//	}
}
}

}
}

void recv(const geometry_msgs::Twist::ConstPtr& msg) {
g_twist_cmd = *msg;
}

void timerCallbackAuto(const ros::TimerEvent& event) {
publishAutonomous();
}
void timerCallbackMain(const ros::TimerEvent& event) {

static tf::TransformBroadcaster broadcaster;

Read(g_port);
Read(g_port2);

//	ROS_INFO("VLeft: %f, VRight: %f", vLeft, vRight);
left_angle = ts * vLeft;
right_angle = ts * vRight;

double vf = WHEEL_RADIUS / 2 * (vRight + vLeft);
double pdot = WHEEL_RADIUS / WHEEL_SPACING * (vRight - vLeft);
veh_speed_twist.header.stamp = event.current_real;
veh_speed_twist.header.frame_id = "/odom";
veh_speed_twist.twist.linear.x = vf;
veh_speed_twist.twist.angular.z = pdot;

double cpsi = (robot_pose.orientation.w * robot_pose.orientation.w)
- (robot_pose.orientation.z * robot_pose.orientation.z);
double spsi = 2 * robot_pose.orientation.w * robot_pose.orientation.z;
double psi = atan2(spsi, cpsi);

robot_pose.position.x += ts * vf * cpsi;
robot_pose.position.y += ts * vf * spsi;
psi += ts * pdot;
robot_pose.orientation = tf::createQuaternionMsgFromYaw(psi);

geometry_msgs::TransformStamped veh2map;
veh2map.header.frame_id = "/odom";
veh2map.header.stamp = event.current_real;
veh2map.child_frame_id = "/base_link";
veh2map.transform.translation.x = robot_pose.position.x;
veh2map.transform.translation.y = robot_pose.position.y;
veh2map.transform.translation.z = robot_pose.position.z;
veh2map.transform.rotation = robot_pose.orientation;
broadcaster.sendTransform(veh2map);

////////////////////Odom msgs ////////////////
odom_msg.header.frame_id = "/odom";
odom_msg.child_frame_id = "/base_link";
odom_msg.pose.pose.position.x = robot_pose.position.x;
odom_msg.pose.pose.position.y = robot_pose.position.y;
odom_msg.pose.pose.position.z = robot_pose.position.z;
odom_msg.pose.pose.orientation = robot_pose.orientation;

odom_msg.twist.twist.linear.x = veh_speed_twist.twist.linear.x;
odom_msg.twist.twist.angular.z = robot_pose.position.z;
odom_msg.header.stamp = event.current_real;
odom_pub.publish(odom_msg);

////////////////////// end Odom///////////////////

twist_pub.publish(veh_speed_twist);

//	Write(g_port);
//	Write(g_port2);
}

int main(int argc, char **argv) {
ros::init(argc, argv, "robot");
ros::NodeHandle n("robot/");

ros::NodeHandle nh_ns("~");
std::string port_name;
std::string port_name2;
nh_ns.param("port", port_name, std::string("/dev/ttyACM0"));
nh_ns.param("port", port_name2, std::string("/dev/ttyACM1"));

ts = 1 / LOOP_RATE;
robot_pose.orientation.w = 1.0;

// Publishers
auto_pub = n.advertise<std_msgs::Bool>("auto", 1, false);	// "robot/auto"
twist_pub = n.advertise<geometry_msgs::TwistStamped>("veh_twist", 1, false);// "robot/actual"
odom_pub = n.advertise<nav_msgs::Odometry>("veh_odom", 1, false);
SerialPort port((char*) port_name.c_str(), BAUD_RATE);
SerialPort port2((char*) port_name2.c_str(), BAUD_RATE);
g_port = &port;
g_port2 = &port2;
ros::Subscriber CmdSub;
//	vLeft = 0.50;
//	vRight = 0.50;
//	if(true){
if (port.isOpen() && port2.isOpen()) {
ROS_INFO("Serial port %s opened successfully.", port_name.c_str());
ROS_INFO("Serial port %s opened successfully.", port_name2.c_str());

ros::Subscriber cmd_sub = n.subscribe("/cmd_vel", 1, recv);	// "/cmd_vel"

ros::Timer auto_timer = n.createTimer(ros::Duration(1 / 2.0),
timerCallbackAuto);	// 2Hz
ros::Timer main_timer = n.createTimer(ros::Duration(1 / LOOP_RATE),
timerCallbackMain);

// Service ros events like timer and message callbacks until shutdown
ros::spin();

g_twist_cmd.linear.x = 0;
g_twist_cmd.angular.z = 0;
Write(&port);
port.closePort();
port2.closePort();
printf("\nClosed\n");
} else {
ROS_ERROR("Failed to open serial port");
}

return 0;
}