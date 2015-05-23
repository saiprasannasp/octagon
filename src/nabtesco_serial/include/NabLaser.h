#ifndef __NABLASER_H__
#define __NABLASER_H__

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

class NabLaser
{
	tf::TransformBroadcaster m_broadcaster;
public:
	NabLaser(ros::NodeHandle& n);
	~NabLaser();

	void BroadcastTransform();
};

#endif