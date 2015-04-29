#ifndef PRFABMAP_H
#define PRFABMAP_H

#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/contrib/openfabmap.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "place_recognizer/keyframeImg.h"
#include "place_recognizer/keyframeMatchInfo.h"

class PRfabmap
{
public:
  PRfabmap(); 	// Initializes FabMap by loading vocabulary, training data etc
  ~PRfabmap();  // Prints the confusion matrix to file before exiting;
 
 void addNewKeyframeBOW(place_recognizer::keyframeImg& kf_msg, int camId); // Computes BOW for each new keyframe and stores it
 void compareKeyframeBOW(cv::Mat bow,int camId); // Compares BOW from two cameras and finds potential overlap.  
 void publishMatchInfo(cv::Mat matchMat , int camId, int fId); // if match is found it publishes on a topic
 //void computeConfusionMat(); // For debugging
 bool isValid() const; // Returns if the class is initialized correctly (i.e. if the required files coul be loaded). 
  
private:  
  
  ros::NodeHandle nh;
  ros::Subscriber sub_kf1;
  ros::Subscriber sub_kf2;
  ros::Publisher pub_match;
  
  void kfCb1(const place_recognizer::keyframeImg::ConstPtr& fmsg);
  void kfCb2(const place_recognizer::keyframeImg::ConstPtr& fmsg);
    
 
  void computeKeyPointLocs(std::vector<cv::KeyPoint>& kpts, cv::Mat& p2d);
  void computeKeyPoint3d(cv::Mat& dMap ,cv::Mat& p2d,cv::Mat& p3d, int camId);
  
  
  cv::Ptr<cv::FeatureDetector> detector;
  cv::Ptr<cv::DescriptorExtractor> extractor;
  cv::Ptr<cv::DescriptorMatcher> matcher;
  cv::Ptr<cv::BOWImgDescriptorExtractor> bide;
  cv::Ptr<cv::of2::FabMap> fabMap;
 
  std::vector<std::vector<cv::KeyPoint> > kptsVec1;
  std::vector<std::vector<cv::KeyPoint> > kptsVec2;
  std::vector<cv::Mat> X2dC1; // 2d feature locations for camera 1
  std::vector<cv::Mat> X2dC2; // 2d feature locations for camera 2
  std::vector<cv::Mat> X3dC1; // 3d feature locations for camera 1
  std::vector<cv::Mat> X3dC2; // 3d feature locations for camera 2
 
  // Camera 1 parameters 
  float fx1;
  float fy1;
  float cx1;
  float cy1;
  unsigned int height1;
  unsigned int width1;
    
  // Camera 2 parameters
  float fx2;
  float fy2;
  float cx2;
  float cy2;
  unsigned int height2;
  unsigned int width2;

  
 
  cv::Mat bow1; // Mat to store bag of words from camera 1
  cv::Mat bow2; // Mat to store bag of words from camera 2
  std::vector<int> fId1;
  std::vector<int> fId2;
  cv::Mat confusionMat;
  double matchThresh; //matching threshold probability to be considered as the same place 
  
  std::string basePath; // base path of this package
  bool valid; // Is FabMap initialized properly or not
  
  
};

#endif // PRFABMAP_H
