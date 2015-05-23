#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Point32.h"
#include <NabSerial.h>
#include <nabtesco_serial/svel.h> 
#include <DiffDriveUtils.h>
#include <tf/transform_listener.h>
#include <NabOdom.h>
#include <NabLaser.h>

//for arduino it all about RPM. so these values whould be RPM
std_msgs::Float32 lvelset, rvelset;

void setPoint(const geometry_msgs::Point32::ConstPtr& msg)
{
   ROS_INFO("I heard: x [%f] y [%f]", msg->x, msg->y);
   lvelset.data = msg->x;
   rvelset.data = msg->y;
}

void cmdVel(const geometry_msgs::Twist::ConstPtr& msg)
{
    DiffDriveUtils::convertTwistToRPM(*msg, lvelset, rvelset);
}

/*void setVelocity(nabtesco_serial::svel::Request& req, nabtesco_serial::svel::Response& resp)
{
  velset = req.lvel;
}*/

int main(int argc, char **argv)
{
  lvelset.data=rvelset.data=0.0;
  
  ros::init(argc, argv, "nabtesco_serial");
  ros::NodeHandle n;
  std::string lusb_name, rusb_name;
  n.param("/nabtesco_serial/lwheel_usb_name", lusb_name, std::string("no device"));
  n.param("/nabtesco_serial/rwheel_usb_name", rusb_name, std::string("no device"));
  double wr=0.0, wb=0.0;
  n.getParam("/nabtesco/wheel_radius", wr);
  n.getParam("/nabtesco/wheel_base", wb);
  DiffDriveUtils::m_wheel_raduis = wr;
  DiffDriveUtils::m_wheel_base = wb;

  nabtesco::NabSerial lnserial(lusb_name);
  nabtesco::NabSerial rnserial(rusb_name);
    
  ros::Publisher lwheelVel = n.advertise<std_msgs::Float32>("lwheelVelocity", 1000);
  ros::Publisher rwheelVel = n.advertise<std_msgs::Float32>("rwheelVelocity", 1000);

  ros::Subscriber sub = n.subscribe("setpoint", 1000, setPoint);
  ros::Subscriber sub_cmdvel = n.subscribe("cmd_vel", 1000, cmdVel);

  NabOdom nab_odom(n);
  //NabLaser nab_laser(n);

  //ros::ServiceServer velocity = n.advertiseService("setVelocity", setVelocity)

  ros::Rate loop_rate(10);
  
  lnserial.Init();
  rnserial.Init();
  while (ros::ok())
  {
    std_msgs::Float32 ltvel, rtvel;
    rtvel.data = ltvel.data = 0.0;
    DiffDriveUtils::convertRPMToVelocity(lnserial.GetVelocity(),ltvel.data);
    DiffDriveUtils::convertRPMToVelocity(rnserial.GetVelocity(),rtvel.data);
    //ROS_INFO("lwhel %f", ltvel.data);
    lwheelVel.publish(ltvel);
    rwheelVel.publish(rtvel);
    lnserial.WriteSerial(lvelset.data);
    rnserial.WriteSerial(rvelset.data);
    ros::spinOnce();
    nab_odom.Update(ltvel.data, rtvel.data);
    //nab_laser.BroadcastTransform();
    loop_rate.sleep();
  }

  return 0;
}