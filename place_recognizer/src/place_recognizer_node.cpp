#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include "prfabmap.h"
#include "place_recognizer/keyframeImg.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
 
static const std::string OPENCV_FEATURES_WINDOW = "Image features window";
cv::Mat bow;
PRfabmap pr_fabmap;

void testCb(const place_recognizer::keyframeImg::ConstPtr& fmsg)
{
  ROS_INFO("Test image callback function : Image Number = %d", fmsg->id);
  // Convert image into cv::Mat via cv_bridge
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(fmsg->img, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
 
  /*
  cv::Mat frame;
  cv_ptr->image.copyTo(frame);
  
  // Compute BOW 
  pr_fabmap.addNewKeyframeBOW(frame, camID);  
  
  // Check OpenFabMap Result -- Debugging version
  if(img->header.seq == 5)
    pr_fabmap.compareKeyframeBOW();
 */
  
}


 
void imageCb1(const place_recognizer::keyframeImg::ConstPtr& fmsg)
{
  int camID = 1;
  ROS_INFO("Image Callback Function 1 : Reading Image Number = %d", fmsg->id);
  // Convert into cv::Mat via cv_bridge
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(fmsg->img, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
 
  cv::Mat frame;
  cv_ptr->image.copyTo(frame);
  
  // Compute BOW 
  pr_fabmap.addNewKeyframeBOW(frame, camID);  
  
  // Comparison done in other callback of camera 2
 
}

void imageCb2(const place_recognizer::keyframeImg::ConstPtr& fmsg)
{

  int camID = 2;
  ROS_INFO("Image Callback Function 2 : Reading Image Number = %d", fmsg->id);
  // Convert into cv::Mat via cv_bridge
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(fmsg->img, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
 
  cv::Mat frame;
  cv_ptr->image.copyTo(frame);
  
  // Compute BOW 
  pr_fabmap.addNewKeyframeBOW(frame, camID); 
  
  // Compare with BOW from camera 1 and check for matches
  pr_fabmap.compareKeyframeBOW();
  
  // If match found, publish result
  
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "place_recognizer_node");
  ros::NodeHandle n;
  ros::Subscriber sub_img1 = n.subscribe("keyframeImgs1", 1, imageCb1);
  ros::Subscriber sub_img2 = n.subscribe("keyframeImgs2", 1, imageCb2);
  ros::Subscriber sub_img3 = n.subscribe("keyframeImgs3", 1, testCb);
  
  ros::spin();

  return 0;
}