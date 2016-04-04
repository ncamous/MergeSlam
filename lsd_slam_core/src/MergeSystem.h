/**

This file is a part of Co-SLAM Project.
Author: Nived Chebrolu
Date: June, 2015

The aim of this class is to manage the merge of LSD-SLAM outputs from two cameras.  
This class will :
1) Reconstruct pose-graph maps from two cameras.
2) Merge two pose-graph maps at keyframes (scene overlap) specified by visual matching module.
3) Compute transformation netween overlapping keyframes using Sim3Tracker (Initialized with an approximate provided by Horn's Method)
4) Output the merged global map to the visualizer.
**/

#pragma once
#include "ros/ros.h"
#include <vector>
#include "DataStructures/Frame.h"
#include "GlobalMapping/KeyFrameGraph.h"
#include "Tracking/TrackingReference.h"
#include "Tracking/Sim3Tracker.h"
#include "lsd_slam_viewer/keyframeGraphMsg.h"
#include "lsd_slam_viewer/keyframeMsg.h"
#include "coslam_msgs/keyframeMatchInfo.h"
#include "KeyFrameGraphDisplay.h"
#include "SlamSystem.h"
namespace lsd_slam{
  
class Frame;
class KeyFrameGraph;
class Sim3Tracker;
class SlamSystem;
class TrackingReference;
class Output3DWrapper;
class KeyFrameGraphDisplay;
struct MergeInitParam{
int w;
int h;
Eigen::Matrix3f K;
};
  
class MergeSystem{

  
public:
 
  // Settings. Constant from construction onward.
  int width;
  int height;
  Eigen::Matrix3f K;
  int nCams=2; // Total number of cameras
  int camId;
  //KeyGraph construction params 
  float downweightFac = 5; 
  const float kernelDelta = 5 * sqrt(6000*loopclosureStrictness);
  int count=0;
  int countKF=0;
  double time_counter = 0;
  bool haveOptOffset=false;
  int numConst=0;
  int numConstPrev=0;	
  clock_t this_time =clock();
  clock_t last_time = this_time;
  int camID;
  bool isIn=false;
  bool doneOpt=false;
  bool keepGoing=false;
  bool enabled=false;
  //double deltaT= (double)(CLOCKS_PER_SEC/3)*2.1;
  double deltaT= (double)(CLOCKS_PER_SEC)/(CLOCKS_PER_SEC); //essay for lsd1. Too bid the delay if not throws sophus exception

  // Initialization parameters (Use if different for two cameras)
  MergeInitParam p1;
  MergeInitParam p2;
    
  MergeSystem(int w, int h, Eigen::Matrix3f K, int nCams);
  //MergeSystem(MergeInitParam p1, MergeInitParam p2, int nCams);
  ~MergeSystem();
   
  void startLiveMerge();
  
  // Read info from each lsd-slam
  bool optimizationIteration(int itsPerTry, float minChange);
  void optimizationThread();
  void mappingThread();
  void optimizationUpdate();
  bool doMappingIteration();
  void keyFrameCb(lsd_slam_viewer::keyframeMsgConstPtr msg);
  void graphCb(lsd_slam_viewer::keyframeGraphMsgConstPtr msg); 
  void matchCb(coslam_msgs::keyframeMatchInfoConstPtr msg);
  
  // Compute Sim3 transformation between two frames (Using Sim3 Tracker)
  void computeTransSim3(int idA, int idB); // Computes Sim3 Transformation given two frame Id's 
  
  // Debug Functions
  void currentKeyFrame_debug();
  void printSim3(Sim3 pose);
  //void graphCb1Debug(lsd_slam_viewer::keyframeGraphMsgConstPtr msg);
 // void buildGraph();

 
  
private:
   
  ros::NodeHandle nh;
  ros::Subscriber keyFrames_sub;
  ros::Subscriber graph_sub;
  ros::Subscriber keyFrames_sub2;
  ros::Subscriber graph_sub2;
  ros::Subscriber match_sub;
  // Frame Pointers
  //std::shared_ptr<Frame> currentKeyFrame;
   std::vector<std::shared_ptr<Frame> > currentKeyFrame;

   Frame* parentKeyFrame;
   
  // Sim3 Sim3Tracker 
  Sim3Tracker* tracker;
  SlamSystem*  Optimizer;

  

// Keyframe Graph and graph related ADDED

  std::vector<KeyFrameGraph*> keyFrameGraph;	// has own locks
  std::vector<SlamSystem*> SLAMSystem;	// has own locks

// std::vector<KFConstraintStruct*, Eigen::aligned_allocator<KFConstraintStruct*> > constraints_;
  std::vector<std::shared_ptr<KFConstraintStruct> >constraints_;	
  std::vector<KFConstraintStruct*, Eigen::aligned_allocator<KFConstraintStruct*> > Constraint;
  std::vector<KFConstraintStruct*, Eigen::aligned_allocator<KFConstraintStruct*> > constraintMatch;
  std::deque< Frame* > newKeyFrames;

	// Optimization thread required for the graph
	boost::shared_mutex edgesListsMutex;
	boost::shared_mutex poseConsistencyMutex;
	bool newConstraintAdded=false;
	boost::mutex newConstraintMutex;
	boost::mutex unmappedTrackedFramesMutex;
	boost::condition_variable newConstraintCreatedSignal;
	boost::condition_variable newFrameMappedSignal;
	boost::condition_variable unmappedTrackedFramesSignal;
	boost::mutex g2oGraphAccessMutex;
	boost::mutex newFrameMappedMutex;
	boost::thread thread_optimization;
	boost::thread thread_mapgraph;
	boost::mutex newKeyFrameMutex;
	boost::condition_variable newKeyFrameCreatedSignal;

        



  
  // Output Viewer
  Output3DWrapper* outputWrapper;	// no lock required

  // Debug variables
  int  count1;
  
 /* int  camID;
  bool isIn= false;
  bool needToPublish= false;
  int  numConst;
  GraphFramePose* framePoseData;
  Sim3 poseKeyFrame;
  GraphConstraint* constraint;

*/


  
};  
  
  
}
