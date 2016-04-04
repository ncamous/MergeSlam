#include "ros/ros.h"
#include "MergeSystem.h"
#include "lsd_slam_viewer/keyframeMsg.h"
#include "MergeSystem.h"

using namespace lsd_slam;
bool haveInitInfo = false;
bool haveInitInfo2 = false;

int w;
int h;
bool status=false;
Eigen::Matrix3f K;
//boost::thread thread_optimization;
//boost::thread thread_mapgraph;
boost::thread check_thread_;
boost::mutex check_mutex;
ros::Time timePrevCheck;

MergeInitParam paramC1;
MergeInitParam paramC2;

void initCb(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
  // Get Initialization Information (h,w,K)
  paramC1.w = msg->width;
  paramC1.h = msg->height;
  float fx = msg->fx;
  float fy = msg->fy;
  float cx = msg->cx;
  float cy = msg->cy;     
  paramC1.K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
  haveInitInfo = true;
  
  //Assuming both cameras have same initial parameters
  w = msg->width;
  h = msg->height;
  K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
   
}


void initCb2(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
  // Get Initialization Information (h,w,K)
  paramC2.w = msg->width;
  paramC2.h = msg->height;
  float fx = msg->fx;
  float fy = msg->fy;
  float cx = msg->cx;
  float cy = msg->cy;     
  Sophus::Matrix3f K_sophus;
  paramC2.K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
  haveInitInfo2 = true;
  
}


void CheckThread(void)
{
  ros::Duration deltaTCheck;
  ros::Rate rate(1);
  ros::NodeHandle check_nh_;//XML postion update ros handler
  
  timePrevCheck=ros::Time::now();

  while (check_nh_.ok())
  {
    //Get shared variables between threads throgh a semaphore
    check_mutex.lock();
    deltaTCheck=ros::Time::now()-timePrevCheck;
    check_mutex.unlock();
    if (deltaTCheck.toSec()>2.0)
    {
        status=false;
    }
    else
    {
        status=true;
    }

    rate.sleep();
  }
  ros::waitForShutdown();
}



int main(int argc, char** argv){
  
  ros::init(argc, argv, "LSD_MERGE");
  ros::NodeHandle nh;
    
  // Read initialization parameters from each camera 
  ros::Subscriber int_sub =  nh.subscribe("/lsd1/lsd_slam/keyframes", 10, initCb);
  ros::Subscriber int_sub2 =  nh.subscribe("/lsd2/lsd_slam/keyframes", 10, initCb2);
  
  int nCams = 2; // Total number of cameras to merge
  
  std::cout<<"Waiting for camera initiliazation parameters."<<std::endl;
  while(ros::ok())
  {  
	  while(!haveInitInfo || !haveInitInfo2)
		{ 
			ros::spinOnce();
		}
  
  std::cout<<"Camera parameters initialized with :"<<std::endl;
  std::cout<<"width = "<< w << "height = "<< h << std::endl << "K = "<< K <<std::endl; 
  
	// MergeSystem merge(paramC1,paramC2); (Use if different initialization parameters)
	MergeSystem merge(w,h,K,nCams);
	merge.startLiveMerge();

  
  ros::spin();

  }
  
  return 0;
  
}
