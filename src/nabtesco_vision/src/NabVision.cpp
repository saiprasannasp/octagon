#include <NabVision.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <ueye/CameraNode.h>

using namespace cv;
using namespace std;
using namespace ueye;

namespace octagon
{

NabVision::NabVision(ros::NodeHandle n, ros::NodeHandle private_n) :
m_cinfo(new camera_info_manager::CameraInfoManager(n))
{
  
  if (m_cinfo->validateURL("package://nabtesco_vision/calibrationdata/ost.ini"))
  {
     m_cinfo->loadCameraInfo("package://nabtesco_vision/calibrationdata/ost.ini");
     ros::ServiceClient client = n.serviceClient<sensor_msgs::SetCameraInfo>("set_camera_info");
     sensor_msgs::SetCameraInfo srv;
     srv.request.camera_info = m_cinfo->getCameraInfo();
    ROS_INFO("Before starting calibration set info");
    /*if (client.call(srv))
    {
      ROS_INFO("Calibration set status %s", srv.response.status_message.c_str());
       if(!srv.response.success) {
          ROS_ERROR("unable to set calibration info to /camera");
       }
    }
    else
    {
      ROS_ERROR("unable to set calibration info to /camera");
      return;
    }*/
     
  }
  else
  {
    // new URL not valid, use the old one
    ROS_ERROR("invalid camera calibration data");
  }

	//ROS_INFO("Inside NabVision");
	m_sub = n.subscribe("/image_raw", 30, &NabVision::HandleImage, this);
  m_subcinfo = n.subscribe("/camera_info", 30, &NabVision::HandleInfo, this);
  m_pubimg = n.advertise<sensor_msgs::Image>("linedetect", 30);
  m_pubcinfo = n.advertise<sensor_msgs::CameraInfo>("cal_camera_info", 30);

  namedWindow("Display window", WINDOW_AUTOSIZE );
  Controls();
}

NabVision::~NabVision()
{

}

void NabVision::HandleInfo(const sensor_msgs::CameraInfo::ConstPtr& cinfo)
{
    sensor_msgs::CameraInfo caminfo = m_cinfo->getCameraInfo();
    caminfo.header = cinfo->header;
    m_pubcinfo.publish(caminfo);
}

void NabVision::DetectLine(cv::Mat& srcimg)
{
  Mat dst, cdst, imgHSV, imgThresholded;
  //ROS_INFO("Inside NabVision Handle");
  Size kernal(3,3);
  cvtColor(srcimg, imgHSV, COLOR_BGR2HSV);
  //cv::GaussianBlur(imgHSV, dst, kernal, 0);
  cv::blur(imgHSV, dst, Size(5, 5), Point(-1, -1));
  cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
  //cv::inRange(imgHSV, const_low, const_high, imgThresholded);
  cv::medianBlur(imgThresholded, imgThresholded, 3);

  //morphological opening (remove small objects from the foreground)
  cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

  //morphological closing (fill small holes in the foreground)
  cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
  cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

  imshow("1", imgHSV);
  imshow("Display window", srcimg);
  imshow("3", imgThresholded);
  cv::waitKey(3);
  //ROS_INFO("Inside NabVision Handle End");
}

void NabVision::Controls()
{
  namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

  iLowH = 70;
  iHighH = 179;

  iLowS = 0; 
  iHighS = 255;

  iLowV = 114;
  iHighV = 255;

  threshold_value = 0;
  threshold_type = 3;;
  max_value = 255;
  max_type = 4;
  max_BINARY_value = 255;


  //Create trackbars in "Control" window
  const char* trackbar_type = "Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted";
  const char* trackbar_value = "Value";

  /// Create Trackbar to choose type of Threshold
  cvCreateTrackbar( trackbar_type,
                  "Control", &threshold_type,
                  max_type);

  cvCreateTrackbar( trackbar_value,
                  "Control", &threshold_value,
                  max_value);

  cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
  cvCreateTrackbar("HighH", "Control", &iHighH, 179);

  cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
  cvCreateTrackbar("HighS", "Control", &iHighS, 255);

  cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
  cvCreateTrackbar("HighV", "Control", &iHighV, 255);
}

void NabVision::HandleImage(const sensor_msgs::Image::ConstPtr& image) 
{
	cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      //ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    DetectLine(cv_ptr->image);
}

void NabVision::Update()
{
    //NODELET_DEBUG("Initializing nodelet...");
    //cv_bridge::CvImagePtr cv_ptr;
    //cv_ptr->image = m_img;
    //m_pubimg.publish(cv_ptr->toImageMsg());
}

}


