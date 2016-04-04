#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// LSD-SLAM Related
#include "DataStructures/Frame.h"
#include "Tracking/TrackingReference.h"
#include "Tracking/Sim3Tracker.h"

// Sophus Related
#include "util/SophusUtil.h"

//COSLAM Related
#include <coslam_msgs/keyframeMatchInfo.h>

#include "ros/ros.h"


// Set Image and Depth Map File Paths:
std::string img1_path = "/home/nicole/coslam_ws/src/coslam/matlab_scripts/frame-22.png";
std::string dep1_path = "/home/nicole/coslam_ws/src/coslam/matlab_scripts/depth-22.png";
std::string img2_path = "/home/nicole/coslam_ws/src/coslam/matlab_scripts/frame-50.png";
std::string dep2_path = "/home/nicole/coslam_ws/src/coslam/matlab_scripts/depth-50.png";

/*std::string img1_path = "/home/nicole/coslam_ws/src/coslam/matlab_scripts/img2.png";
std::string dep1_path = "/home/nicole/coslam_ws/src/coslam/matlab_scripts/dep2.png";
std::string img2_path = "/home/nicole/coslam_ws/src/coslam/matlab_scripts/img270.png";
std::string dep2_path = "/home/nicole/coslam_ws/src/coslam/matlab_scripts/dep270.png";

*/

// Define switch options
//#define DEBUG_INPUT
#define UEYE_FISH



// Calibration Parameters
//Camera Parameters (Xtion)
#ifdef XTION
float f = 570.34;   // Focal length
float cX = 314.5;  // Center X
float cY = 235.5;  // Center Y
float sf = 1000.0; // Scaling Factor
#endif

#ifdef KINECT
// Camera Parameters (Kinect)
float f = 525.0;   // Focal length
float cX = 319.5;  // Center X
float cY = 239.5;  // Center Y
float sf = 5000.0; // Scaling Factor
#endif

#ifdef UEYE_FISH
float f =  300;   // Focal length
float cX = 320;  // Center X
float cY = 240;  // Center Y
float sf = 1; // Scaling Factor
#endif

// Function Prototypes
void printSim3(Sim3 pose);
void computeIdepth(cv::Mat& dep, cv::Mat& idep, cv::Mat& idep_var);

// Namespace Definition
using namespace lsd_slam;

int main(int argc, char** argv){
 
  ros::init(argc, argv, "SIM3 TRACK");
  ros::NodeHandle nh;
  
  ros::Publisher pub=nh.advertise<coslam_msgs::keyframeMatchInfo>("/matchInfo",1);
 
 // ros::spin();
  
  
 
 //----------------- Load Images + Corresponding DepthMap -------------------------------------------- //
    cv::Mat img1 = cv::imread(img1_path, CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat dep1_gray = cv::imread(dep1_path, CV_LOAD_IMAGE_GRAYSCALE);
    //cv::Mat dep1 = cv::imread(dep1_path, CV_LOAD_IMAGE_UNCHANGED);    
    cv::Mat img2 = cv::imread(img2_path, CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat dep2_gray = cv::imread(dep2_path, CV_LOAD_IMAGE_GRAYSCALE);
    //cv::Mat dep2 = cv::imread(dep2_path, CV_LOAD_IMAGE_UNCHANGED);    



//Convert Mat type from 8UC1 to 16UC1
cv::Mat dep1;
dep1_gray.convertTo(dep1,CV_16UC1,(65535)/255.0,0.0);

cv::Mat dep2;
dep2_gray.convertTo(dep2,CV_16UC1,(65535)/255.0,0.0);


#ifdef DEBUG_INPUT
    // Visualize Images and depth map
    cv::namedWindow("Image1");
    cv::imshow("Image1",img1);
    cv::waitKey(0);
    cv::namedWindow("Image2");
    cv::imshow("Image2",img2);
    cv::waitKey(0);
    cv::namedWindow("Depth1");
    cv::imshow("Depth1",dep1);
    cv::waitKey(0);
    cv::namedWindow("Depth2");
    cv::imshow("Depth2",dep2);
    cv::waitKey(0);
#endif 
    
    
//--------------------- Construct LSD-SLAM keyframe structure ------------------------------------------- //    
  
  // Frame Metadata   
  int id = 1;
  double timestamp = 1.0;
  int w = img1.cols;
  int h = img1.rows;
  
  Sophus::Matrix3f K_sophus;
  K_sophus << f, 0.0, cX, 0.0, f, cY, 0.0, 0.0, 1.0;
 
  // Camera Calibration Matrix
  std::cout<<"------- Camera Calibration Matrix ----------------------------------"<<std::endl;
  std::cout<<K_sophus<<std::endl;  
  std::cout<<"--------------------------------------------------------------------"<<std::endl;
   
  
  // Fill with Image and Depth Data  
  cv::Mat idep1 = cv::Mat(h,w,CV_32F);
  cv::Mat idep1_var = cv::Mat(h,w,CV_32F);

//  cv::Mat idep1, idep1_var;
  Sim3 KF1_pose = Sim3();
  // Keyframe 1
  Frame* KF1 = new Frame(id, w, h, K_sophus, timestamp, img1.data);
  computeIdepth(dep1,idep1, idep1_var);  //builds depth and inverse depth of image 
  KF1->setIDepthKeyFrame(idep1.data, idep1_var.data); //computes depth only in level zero needed to reconstruct 3D points
  KF1->buildAllPyramidLevels(); 
  KF1->pose->setPoseExternal(KF1_pose);
   
  //Keyframe 2
  cv::Mat idep2 = cv::Mat(h,w,CV_32F);
  cv::Mat idep2_var = cv::Mat(h,w,CV_32F);
//  cv::Mat idep2, idep2_var;
  Sim3 KF2_pose = Sim3();
  Frame* KF2 = new Frame(id, w, h, K_sophus, timestamp, img2.data); 
  computeIdepth(dep2,idep2, idep2_var); 
  KF2->setIDepthKeyFrame(idep2.data, idep2_var.data);
  KF2->buildAllPyramidLevels();
  KF2->pose->setPoseExternal(KF2_pose); 
  
  
  //---------------------- Debugging Space ------------------------------------------------------------------//
 /* std::cout<<"Depth Image Type = "<< dep1.type() << std::endl;  
  
    //
  // Keyframe Image Debug
     int lvl = 0;
     cv::Mat image_debug = cv::Mat(KF2->height(lvl), KF2->width(lvl),CV_8U);
     const float* pt1 = KF2->image(lvl);
     for(int i =0 ; i< KF2->height(lvl) ; i++) // For each row
     {
        uchar* data_image_debug = image_debug.ptr<uchar>(i);
             
	for(int j = 0; j < KF2->width(lvl) ; j++) // for each column
	{  
	  data_image_debug[j] = (uchar)*pt1; 
	  pt1++;
	}
     }

     cv::imshow("Image debug Frame",image_debug);
     cv::waitKey(0);
       
     cv::Mat idepth_debug = cv::Mat(KF2->height(lvl), KF2->width(lvl),CV_32F);
     const float* pt2 = KF2->idepth(lvl);
     for(int i =0 ; i< KF2->height(lvl) ; i++) // For each row
     {
        float* data_idepth_debug = idepth_debug.ptr<float>(i);
             
	for(int j = 0; j < KF2->width(lvl) ; j++) // for each column
	{  
	  data_idepth_debug[j] = *pt2; 
	  pt2++;
	}
     }

     
         
   std::cout<<"-------------------------------------------------------------------------------------------"<<std::endl;
     std::cout<<"IDepth" <<idepth_debug(cv::Range(200,210),cv::Range(200,210)) <<std::endl;
     std::cout<<"-------------------------------------------------------------------------------------------"<<std::endl;
     
     std::cout<<"-------------------------------------------------------------------------------------------"<<std::endl;
     std::cout<<"Depth" <<dep2(cv::Range(200,210),cv::Range(200,210)) <<std::endl;
     std::cout<<"-------------------------------------------------------------------------------------------"<<std::endl;
    
     
     cv::imshow("IDepth Debug Frame",idepth_debug);
     cv::waitKey(0); 
   */  
  
  Eigen::Matrix<double,4,4> T;
  T << 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1, 0,
       0, 0, 0, 1;

  
  /*     
  T << 0.97453219, -0.21718283, 0.16654855, 0.0025491067,
  0.2410147, 0.97287035, -0.14161539, -0.16890989,
  -0.129687, 0.17599611, 0.98834527, -0.26565069,
  0, 0, 0, 1;
     
       
       
       
   T <<  0.870678,   -0.491839, -0.00374116, -0.00336255,
	0.491828,    0.870685,  -0.0035236,  0.00564547,
	0.00499042,  0.00122792  , 0.999987, -0.00212246,
        0      ,     0  ,         0   ,        1 ;
  
   T << 0.87, 0.50, -0.09, -0.00,
  -0.51, 0.87, -0.04, -0.00,
  0.05, 0.09, 1.00, -0.00,
  0, 0, 0,1;
  
  
  std::cout<<"-------------------------------------------------------------------------------------------"<<std::endl;
  std::cout<<"Eigen Matrix"<< std::endl << T << std::endl;
  std::cout<<"-------------------------------------------------------------------------------------------"<<std::endl;
  */
  
  //----------------------- Call Sim3Tracker Class -----------------------------------------------------------//    
  Sim3Tracker* tracker = new Sim3Tracker(w,h,K_sophus);
  tracker->settings.maxItsPerLvl[0] = 1000;
  tracker->settings.maxItsPerLvl[1] = 1000;
  tracker->settings.maxItsPerLvl[2] = 1000;
  tracker->settings.maxItsPerLvl[3] = 1000;
  tracker->settings.maxItsPerLvl[4] = 1000;
  
  // Set from Horn's Estimate
  Sim3 T12_init = Sim3(T); 
  Sim3 T12_final; 
  
  TrackingReference* tr1 = new TrackingReference();  
  tr1->importFrame(KF1);  
    
  
  // Estimate: Before Optimization 
  std::cout<<"-------------------------------------------------------------------------------------------"<<std::endl;
  std::cout<<"Before Optimization :"<< std::endl;
  printSim3(T12_init);
  std::cout<<"-------------------------------------------------------------------------------------------"<<std::endl;
  
  // At Different Start And End Levels   
  int startLvl = 4; int endLvl = 3;
  T12_final = tracker->trackFrameSim3(tr1, KF2, T12_init, startLvl, endLvl);  
  std::cout<<"-------------------------------------------------------------------------------------------"<<std::endl;
  std::cout<<"After Optimization :"<< std::endl;
  std::cout<<"At Start Level = "<< startLvl << ", End Level = " << endLvl << std::endl;
  printSim3(T12_final);
  std::cout<<"-------------------------------------------------------------------------------------------"<<std::endl;


  //------------------------- Tracker Debugging Info. ------------------------------------------------
  std::cout<<"---------------------------------------------------------------------------------------------"<<std::endl;
  std::cout<<"Sim3Tracker Debuging Information"<<std::endl;
  std::cout<<"Last Sim3 Hessian = "<< std::endl <<tracker->lastSim3Hessian <<std::endl;
  std::cout<<"Last Residual = "<< tracker->lastResidual <<std::endl;
  std::cout<<"Last Depth Residual = "<< tracker->lastDepthResidual <<std::endl;
  std::cout<<"Last Photometric Residual = "<< tracker->lastPhotometricResidual <<std::endl;
  std::cout<<"Point Usage = "<< tracker->pointUsage <<std::endl;
  std::cout<<"Sim3 Tracker Diverged = "<< tracker->diverged <<std::endl;
  std::cout<<"---------------------------------------------------------------------------------------------"<<std::endl;
  
  
  startLvl = 2;  endLvl = 2;
  T12_final = tracker->trackFrameSim3(tr1, KF2, T12_init, startLvl, endLvl);  
  std::cout<<"-------------------------------------------------------------------------------------------"<<std::endl;
  std::cout<<"After Optimization :"<< std::endl;
  std::cout<<"At Start Level = "<< startLvl << ", End Level = " << endLvl << std::endl;
  printSim3(T12_final);
  std::cout<<"-------------------------------------------------------------------------------------------"<<std::endl;
  
  //------------------------- Tracker Debugging Info. ------------------------------------------------
  std::cout<<"---------------------------------------------------------------------------------------------"<<std::endl;
  std::cout<<"Sim3Tracker Debuging Information"<<std::endl;
  std::cout<<"Last Sim3 Hessian = "<< std::endl <<tracker->lastSim3Hessian <<std::endl;
  std::cout<<"Last Residual = "<< tracker->lastResidual <<std::endl;
  std::cout<<"Last Depth Residual = "<< tracker->lastDepthResidual <<std::endl;
  std::cout<<"Last Photometric Residual = "<< tracker->lastPhotometricResidual <<std::endl;
  std::cout<<"Point Usage = "<< tracker->pointUsage <<std::endl;
  std::cout<<"Sim3 Tracker Diverged = "<< tracker->diverged <<std::endl;
  std::cout<<"---------------------------------------------------------------------------------------------"<<std::endl;
  
  
  startLvl = 1;  endLvl = 1;
  T12_final = tracker->trackFrameSim3(tr1, KF2, T12_init, startLvl, endLvl);  //tracking reference, frame, initial guess, start and end level pyramid
  std::cout<<"-------------------------------------------------------------------------------------------"<<std::endl;
  std::cout<<"After Optimization :"<< std::endl;
  std::cout<<"At Start Level = "<< startLvl << ", End Level = " << endLvl << std::endl;
  printSim3(T12_final);
  std::cout<<"-------------------------------------------------------------------------------------------"<<std::endl;
  
 
  //------------------------- Tracker Debugging Info. ------------------------------------------------
  std::cout<<"---------------------------------------------------------------------------------------------"<<std::endl;
  std::cout<<"Sim3Tracker Debuging Information"<<std::endl;
  std::cout<<"Last Sim3 Hessian = "<< std::endl <<tracker->lastSim3Hessian <<std::endl;
  std::cout<<"Last Residual = "<< tracker->lastResidual <<std::endl;
  std::cout<<"Last Depth Residual = "<< tracker->lastDepthResidual <<std::endl;
  std::cout<<"Last Photometric Residual = "<< tracker->lastPhotometricResidual <<std::endl;
  std::cout<<"Point Usage = "<< tracker->pointUsage <<std::endl;
  std::cout<<"Sim3 Tracker Diverged = "<< tracker->diverged <<std::endl;
  std::cout<<"---------------------------------------------------------------------------------------------"<<std::endl;
  
    while(ros::ok())
 {

  coslam_msgs::keyframeMatchInfo MImsg;
    
  MImsg.isMatch=true;
  MImsg.matchProb=0.9;
  MImsg.iCam= 2;
  MImsg.kfId1= 50;
  MImsg.kfId2=22;
  
  Frame* f_temp= new Frame(id, w, h, K_sophus, timestamp, img1.data);
  ROS_INFO("ok to here");
  
  f_temp->pose->setPoseExternal(T12_final);

//  T12_final->getScaledCamToWorld();
  memcpy(MImsg.BtoA.data(),f_temp->getScaledCamToWorld().cast<float>().data(),sizeof(float)*7);
  

  /*------------CHECKING--------------------*/
  std::cout<<"DEBUG"<<std::endl;
  std::cout<<"rotation"<<T12_final.rotationMatrix()<<std::endl;
  std::cout<<"translation"<<T12_final.translation()[0]<<","<<T12_final.translation()[1]<<","<<T12_final.translation()[2]<<","<<std::endl;
  
  std::cout<<"Pose Quaternion ="<<f_temp->getScaledCamToWorld().quaternion().w()<<","<<f_temp->getScaledCamToWorld().quaternion().x()<< "," << f_temp->getScaledCamToWorld().quaternion().y() <<"," << f_temp->getScaledCamToWorld().quaternion().z() <<std::endl;
  std::cout<<"Pose Translation ="<<f_temp->getScaledCamToWorld().translation()[0]<<","<<f_temp->getScaledCamToWorld().translation()[1] <<","<< f_temp->getScaledCamToWorld().translation()[2] <<std::endl;

  pub.publish(MImsg);
  ros::Rate loop_rate(1);
  loop_rate.sleep();
}
  return 0;  

  
}



void printSim3(Sim3 pose)
{
  //std::cout<<"Pose Quaternion ="<<pose.quaternion().w()<<","<<pose.quaternion().x()<< "," << pose.quaternion().y() <<"," << pose.quaternion().z() <<std::endl;
  std::cout<<"Rotation Matrix ="<<std::endl<< pose.rotationMatrix()<<std::endl;
  std::cout<<"Pose Translation ="<<std::endl<<pose.translation()[0]<<","<< pose.translation()[1] <<","<< pose.translation()[2] <<std::endl;
  std::cout<<"Scale ="<<pose.scale() << std::endl;
 
}

void computeIdepth(cv::Mat& dep, cv::Mat& idep, cv::Mat& idep_var)
{
  
  idep.create(dep.rows, dep.cols, CV_32F);//inverse depth
  idep_var.create(dep.rows, dep.cols, CV_32F);

  std::cout<<"Depth Image Type = " << dep.type() << std::endl;
  std::cout<<"Idepth Image Type = " << idep.type() << std::endl;

  float z = 0.0;  
  for(int i = 0; i < dep.rows; i++)
  { 
    for(int j = 0; j < dep.cols; j++)
    {	
      if(dep.at<short unsigned int>(i,j) > 0)
	{	
	  z = (float)dep.at<short unsigned int>(i,j)/sf;	
	  idep.at<float>(i,j) = 1.0/z;  //inverse depth
	  idep_var.at<float>(i,j) = 0.01*0.01;
	}  
       
    }
  }
 
  /*
    cv::imshow("Depth",dep);
    cv::waitKey(0);
    cv::imshow("IDepth",idep);
    cv::waitKey(0);
  
 
     std::cout<<"-------------------------------------------------------------------------------------------"<<std::endl;
     std::cout<<"Depth" << std::endl<<dep(cv::Range(200,210),cv::Range(200,210)) <<std::endl;
     std::cout<<"-------------------------------------------------------------------------------------------"<<std::endl;
  
     std::cout<<"-------------------------------------------------------------------------------------------"<<std::endl;
     std::cout<<"IDepth ="<< std::endl <<idep(cv::Range(200,210),cv::Range(200,210)) <<std::endl;
     std::cout<<"-------------------------------------------------------------------------------------------"<<std::endl;
  */   
  
}




















