#include <NabLaser.h>

NabLaser::NabLaser(ros::NodeHandle& n)
{

}

NabLaser::~NabLaser()
{

}

void NabLaser::BroadcastTransform()
{
	m_broadcaster.sendTransform( tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)), ros::Time::now(),"base_link", "base_laser"));
}