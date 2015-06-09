#ifndef __NABVISION_H__
#define __NABVISION_H__

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>

namespace octagon
{
  class NabVision
  {
  	ros::Subscriber m_sub, m_subcinfo;
  	ros::Publisher m_pubimg, m_pubcinfo;

     int iLowH;
    int iHighH;

    int iLowS; 
    int iHighS;

    int iLowV;
    int iHighV;

    int threshold_value;
    int threshold_type;
    int max_value;;
    int max_type;
    int max_BINARY_value;

    cv::Mat m_img;

    boost::shared_ptr<camera_info_manager::CameraInfoManager> m_cinfo;

  public:
  	NabVision(ros::NodeHandle node, ros::NodeHandle private_n);
  	~NabVision();

  	void Controls();

  	void DetectLine(cv::Mat& srcimg);

  	void HandleImage(const sensor_msgs::Image::ConstPtr& image);
    void HandleInfo(const sensor_msgs::CameraInfo::ConstPtr& cinfo);

  	void Update();

  };
}
#endif