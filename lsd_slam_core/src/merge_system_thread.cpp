#include "MergeSystem.h"
#include "DataStructures/Frame.h"
#include "GlobalMapping/KeyFrameGraph.h"
#include "util/SophusUtil.h"
#include "opencv2/opencv.hpp"
#include "IOWrapper/ROS/ROSOutput3DWrapper.h"
#include "sophus/sim3.hpp"
 #include <g2o/core/robust_kernel_impl.h>
 #include <time.h>
 #include "deque"


using namespace lsd_slam;

bool MergeSystem::optimizationIteration(int itsPerTry, float minChange)
{

	g2oGraphAccessMutex.lock();
	std::cout<<"Starting"<<std::endl;
		
	if(newConstraintMutex.try_lock())
	{
	//	std::cout<<"Accesing to Lock 1"<<std::endl;
		keyFrameGraph.at(camID)->addElementsFromBuffer();
	//	std::cout<<"Exiting to Lock 1"<<std::endl;
		newConstraintMutex.unlock();
	}	
		else
	{
		std::cout<<"busy"<<std::endl;
		return false;
	}
	
	//keyFrameGraph.at(camID)->graph.initializeOptimization(0);
	int its = keyFrameGraph.at(camID)->optimize(itsPerTry);

	//std::cout<<"optimized"<<std::endl;
	
	
	float maxChange = 0;
	float sumChange = 0;
	float sum = 0;
	
	poseConsistencyMutex.lock_shared();
	keyFrameGraph.at(camID)->keyframesAllMutex.lock_shared();
	//std::cout<<"changes into graph"<<std::endl;
	for(size_t i=0;i<keyFrameGraph.at(camID)->keyframesAll.size(); i++)
	{
		// set edge error sum to zero
		keyFrameGraph.at(camID)->keyframesAll[i]->edgeErrorSum = 0;
		keyFrameGraph.at(camID)->keyframesAll[i]->edgesNum = 0;
		//std::cout<<"OK1"<<std::endl;
		
		if(!keyFrameGraph.at(camID)->keyframesAll[i]->pose->isInGraph) continue;
			

				// get change from last optimization				

				Sim3 a = keyFrameGraph.at(camID)->keyframesAll[i]->pose->graphVertex->estimate();
				Sim3 b = keyFrameGraph.at(camID)->keyframesAll[i]->getScaledCamToWorld();
				Sophus::Vector7f diff = (a*b.inverse()).log().cast<float>();
				//std::cout<<"OK2"<<std::endl;

				for(int j=0;j<7;j++)
				{
					float d = fabsf((float)(diff[j]));
					if(d > maxChange) maxChange = d;
					sumChange += d;
				}
				sum +=7;
				
				
				// set change
				keyFrameGraph.at(camID)->keyframesAll[i]->pose->setPoseGraphOptResult(keyFrameGraph.at(camID)->keyframesAll[i]->pose->graphVertex->estimate());
//				std::cout<<"OK1"<<std::endl;


				// add error
				for(auto edge : keyFrameGraph.at(camID)->keyframesAll[i]->pose->graphVertex->edges())
				{
					keyFrameGraph.at(camID)->keyframesAll[i]->edgeErrorSum += ((EdgeSim3*)(edge))->chi2();
					keyFrameGraph.at(camID)->keyframesAll[i]->edgesNum++;					
				}
	//			std::cout<<"OK2"<<std::endl;
				
		
		}
	haveOptOffset=true;	
	//std::cout<<"Back from optimization"<<std::endl;

	
	keyFrameGraph.at(camID)->keyframesAllMutex.unlock_shared();
	poseConsistencyMutex.unlock_shared();
	g2oGraphAccessMutex.unlock();


//	std::cout<<"going back"<<std::endl;
	
	return maxChange > minChange && its == itsPerTry;
}	


void MergeSystem::optimizationThread()
{
	ROS_INFO("optimizationThread");
	while(keepGoing)
	{
		boost::unique_lock<boost::mutex> lock(newConstraintMutex);
		if(!newConstraintAdded)
			newConstraintCreatedSignal.timed_wait(lock,boost::posix_time::milliseconds(2000));	// slight chance of deadlock otherwise
		newConstraintAdded = false;
		lock.unlock();
		if(isIn)
		{
		doneOpt=true;
		while(optimizationIteration(2, 0.5));
		doneOpt=false;
		}
	}
//	std::cout<<"Exit optimizationThread"<<std::endl;
//		doneOpt=false;
}



//Constructor (Both Cameras with same parameters)
MergeSystem::MergeSystem(int w, int h, Eigen::Matrix3f K,int nCams)
{
  // Initialisation parameters
  this->width = w;
  this->height = h;
  this->K = K;
  this->nCams = nCams;
  
  // Initialisation
  tracker = new Sim3Tracker(w,h,K);
  
  // Initialize Graph Vector 
  //[Graph 0 = Combined Graph, Graph 1 = Camera 1, Graph 2 = Camera 2]
  keyFrameGraph.resize(nCams+1);
  for(int i = 0; i <= nCams ; i++)
  {
    keyFrameGraph.at(i) = new KeyFrameGraph();
  }
 
  // Initialize currentKeyFrame vector
  currentKeyFrame.resize(nCams+1);
  keyFrameGraph.resize(nCams+1);
  
  // Initialize Output Wrapper
  outputWrapper = new ROSOutput3DWrapper(w,h);
  
  // Debug Variables
  count1 = 0;
}



void MergeSystem::optimizationUpdate()
{	
	poseConsistencyMutex.lock();

	bool needToPublish = false;
	if(haveOptOffset)
	{
		keyFrameGraph.at(camID)->keyframesAllMutex.lock_shared();
		for(unsigned int i=0;i<keyFrameGraph.at(camID)->keyframesAll.size(); i++)
			keyFrameGraph.at(camID)->keyframesAll[i]->pose->applyPoseGraphOptResult();
		keyFrameGraph.at(camID)->keyframesAllMutex.unlock_shared();
		//std::cout<<"Updated"<<std::endl;

		haveOptOffset = false;
		needToPublish = true;

	}

	poseConsistencyMutex.unlock();
	//keepGoing=false;
	if (needToPublish)
	{
		//std::cout<<"Published"<<std::endl;
		if (outputWrapper != nullptr)
		{
			outputWrapper->publishKeyframeGraph(keyFrameGraph.at(camID));	
			//std::cout<<"Publishing graph with num constraints:"<<keyFrameGraph.at(camID)->edgesAll.size()<<std::endl;
			//doneOpt=false;
		}
	
	}
	
}


bool MergeSystem::doMappingIteration()
{
	if(currentKeyFrame.at(camId) == 0)
		{
		std::cout<<"empty"<<std::endl;
		return false;
		}
	else
	{
		if(isIn)
		{
			std::cout<<"Going to Update"<<std::endl;
			optimizationUpdate();
			return true;
		}
	}
}


void MergeSystem::mappingThread()
{
	ROS_INFO("MappingThread");
	int count_map=0;

	while(keepGoing)
	{
		count_map++;
//		if(isIn)
	//	{
//			optimizationUpdate();
			boost::unique_lock<boost::mutex> lock(unmappedTrackedFramesMutex);
			if(!haveOptOffset)
				unmappedTrackedFramesSignal.timed_wait(lock,boost::posix_time::milliseconds(200));	// slight chance of deadlock otherwise
			lock.unlock();
			if(isIn)
				optimizationUpdate();
			
		/*	if (!doMappingIteration())
			{
				boost::unique_lock<boost::mutex> lock(unmappedTrackedFramesMutex);
//				lock.try_lock_until(boost::posix_time::milliseconds(200));
				unmappedTrackedFramesSignal.timed_wait(lock,boost::posix_time::milliseconds(200));	// slight chance of deadlock otherwise
				lock.unlock();
			}*/
		//keepGoing=false;
		newFrameMappedMutex.lock();
		newFrameMappedSignal.notify_all();
		newFrameMappedMutex.unlock();
	}
}
//	std::cout<<"Exit MappingThread"<<std::endl;
	//doneOpt=false;

//}



/*//Constructor (Both Cameras with different parameters)
MergeSystem::MergeSystem(MergeInitParam p1, MergeInitParam p2, int nCams)
{
 
  this->p1.w = p1.w;
  this->p1.h = p1.h;
  this->p1.K = p1.K;
  
  this->p2.w = p2.w;
  this->p2.h = p2.h;
  this->p2.K = p2.K;
 
  this->nCams = nCams;
  
  // Initialisation
  tracker = new Sim3Tracker(p1.w,p1.h,p1.K);
  
 // [Graph 0 = Combined Graph, Graph 1 = Camera 1, Graph 2 = Camera 2]
  keyFrameGraph.resize(nCams+1);
  for(int i = 0; i <= nCams ; i++)
  {
    keyFrameGraph.at(i) = new KeyFrameGraph();
  }
  
  
  currentKeyFrame.resize(nCams+1);

 
  // Debug Variables
  count1 = 0;
}

*/

// Destructor
MergeSystem::~MergeSystem()
{
	keepGoing=false;
	doneOpt=false;
	isIn=false;
 // Release Resources
  
  
 // Delete Sim3 Tracker
 // delete tracker;
  
  
 // Clear all shared pointers before deleting the KeyFrameGraph
 //currentKeyFrame.reset();
  
  
 // Delete KeyFrameGraph 
 // delete keyFrameGraph;  
  
  
}


// Defines subscribers to Keyframes and graphs from each camera.
// Later can be used to initialise threads
void MergeSystem::startLiveMerge()
	{

	keepGoing = true;
	ROS_INFO("threads");	
	thread_optimization = boost::thread(&MergeSystem::optimizationThread, this);
	thread_mapgraph= boost::thread(&MergeSystem::mappingThread, this);	
	
/*	for(int i=1; i<nCams;i++)
	{
		keyFrameGraph.at(i)=new KeyFrameGraph();	
		ROS_INFO("Num of new keyframes %d %d",i,nCams);
	}
	*/	keyFrameGraph.at(1)=new KeyFrameGraph();	
		keyFrameGraph.at(2)=new KeyFrameGraph();	

	
	keyFrames_sub = nh.subscribe("/lsd1/lsd_slam/keyframes", 200, &MergeSystem::keyFrameCb, this); //Made bigger the buffer due to large time of GraphCallback 
	graph_sub = nh.subscribe("/lsd1/lsd_slam/graph", 200, &MergeSystem::graphCb, this); //lsd1 while reconstructing graph throws exception of sophus::scalenotpositive rxso3.hpp
  
	keyFrames_sub2 = nh.subscribe("/lsd2/lsd_slam/keyframes", 200, &MergeSystem::keyFrameCb, this);
	graph_sub2 = nh.subscribe("/lsd2/lsd_slam/graph",200, &MergeSystem::graphCb, this);

	match_sub = nh.subscribe("/matchInfo",10, &MergeSystem::matchCb, this);


  //  ros::MultiThreadedSpinner s(2);
	//s.spin();

	ros::AsyncSpinner s(4);//The number is the maximum number of paralell threads
	s.start();
	

    
    
    
}


// Reconstruct keyframe from each lsd-slam
void MergeSystem::keyFrameCb(lsd_slam_viewer::keyframeMsgConstPtr msg)
{


  ROS_INFO("Keyframe Callback");
  std::cout << "Camera Id = " << msg->camId << " ,Current Keyframe Id =" << msg->id << std::endl;
 // keepGoing = true;
  
  // Read Keyframe Message

  camId = msg->camId;
  int id = msg->id;
  double timestamp = msg->time;
  int w = msg->width;
  int h = msg->height;
  float fx = msg->fx;
  float fy = msg->fy;
  float cx = msg->cx;
  float cy = msg->cy;     
  Sophus::Matrix3f K_sophus;
  K_sophus << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;




  
  //  Read pointcloud data (idepth, idepth_var and image)
  InputPointDense* pc = (InputPointDense*)msg->pointcloud.data();
  cv::Mat image = cv::Mat(h,w,CV_8U);
  cv::Mat idepth = cv::Mat(h,w,CV_32F);
  cv::Mat idepth_var = cv::Mat(h,w,CV_32F);
     
  for(int i =0; i<h; i++) // For each row
  {
    uchar* data_img = image.ptr<uchar>(i);
    float* data_idepth = idepth.ptr<float>(i);
    float* data_idepth_var = idepth_var.ptr<float>(i);
       
       for(int j = 0; j<w ; j++) // for each column
	{
	  data_img[j] = pc->color[0];
	  data_idepth[j] = pc->idepth;
	  data_idepth_var[j] = pc->idepth_var;
	  pc++;
	}
  }
      
  // Read Pose of currentKeyFrame 
  std::vector<double> camToWorld(7);
  for(unsigned int i = 0; i < msg->camToWorld.size(); i++)
    {   camToWorld[i]  = (double) msg->camToWorld[i];
        // std::cout<< msg->camToWorld[i]<<",";
    }
  Sim3 poseKeyFrame;
  memcpy(poseKeyFrame.data(),camToWorld.data(), sizeof(double)*7);
  //std::cout<<"Current Keyframe Pose:" << std::endl;
 // printSim3(poseKeyFrame);

        
  // Set keyframe data (image, idepth, idepth_var) + Build all pyramid levels
  currentKeyFrame.at(camId).reset(new Frame(id, w, h, K_sophus, timestamp, image.data));
  currentKeyFrame.at(camId)->setIDepthKeyFrame(idepth.data, idepth_var.data);
  currentKeyFrame.at(camId)->buildAllPyramidLevels(); 
  

  
  // Set Pose
  std::cout<<"Set pose"<<std::endl;
  currentKeyFrame.at(camId)->pose->setPoseExternal(poseKeyFrame);
  //printSim3(currentKeyFrame->pose->getCamToWorld());
  


	keyFrameGraph.at(camId)->addFrame(currentKeyFrame.at(camId).get());



  // Add to keyframesAll(vector)
  std::cout<<"add to vector"<<std::endl;
/*  keyFrameGraph.at(camId)->keyframesAllMutex.lock_shared();
  keyFrameGraph.at(camId)->keyframesAll.push_back(currentKeyFrame.at(camId).get());
  keyFrameGraph.at(camId)->keyframesAllMutex.unlock_shared();
  */
  // Add to idToKeyframe (Unordered_map)
  std::cout<<"Unordered map"<<std::endl;
  keyFrameGraph.at(camId)->idToKeyFrameMutex.lock_shared();
  keyFrameGraph.at(camId)->idToKeyFrame.insert(std::make_pair(currentKeyFrame.at(camId)->id(), currentKeyFrame.at(camId)));
  keyFrameGraph.at(camId)->idToKeyFrameMutex.unlock_shared();

  


  // Output to Viewer
/*	if(camId==1)
    outputWrapper->publishKeyframe(currentKeyFrame.at(camId).get());
	else if(camId==2)
	outputWrapper->publishKeyframe(currentKeyFrame.at(camId).get());
 */
 
  // Debug Space
	//if(camId==1)
      //currentKeyFrame_debug();


  // Image Debug Window
  
  /*if(camId == 1)
  {
    cv::imshow("Image: Cam 1",image);
    cv::waitKey(1);
    cv::imshow("Depth: Cam 1",idepth);
    cv::waitKey(1);
  }
  else
  {
    cv::imshow("Image: Cam 2",image);
    cv::waitKey(1);
    cv::imshow("Depth: Cam 2",idepth);
    cv::waitKey(1);
  }
  */
  	countKF++;
	std::cout<<"Num of KF msgs received:" << countKF <<std::endl;

}



// Reconstruct graph from each lsd-slam
void MergeSystem::graphCb(lsd_slam_viewer::keyframeGraphMsgConstPtr msg)
{
	if(!doneOpt)
	{	
		ROS_INFO("Graph Callback");
		this_time = clock();
		keepGoing = true;
        time_counter += (double)(this_time - last_time);
		std::cout<< time_counter<<std::endl;
		std::cout<<  deltaT<<std::endl;
		Constraint.clear();
		newKeyFrames.clear();
        last_time = this_time;
        isIn=false;
	   	
	if(time_counter > deltaT)
	{
		time_counter = 0;
		count++;
		//std::cout<< "Num of threated Graph msgs: "<<count<<std::endl;
		deltaT=deltaT;	  
		camID=msg->camId;
		numConst=msg->numConstraints;
		//isIn= false;

		std::shared_ptr<Frame> reffirst;
		std::shared_ptr<Frame> refsecond;

		GraphConstraint* constraint=(GraphConstraint*)msg->constraintsData.data();
	//	std::cout<<"num frames received "<<msg->numFrames<<std::endl;
		//std::cout<<"num constraints received "<<msg->numConstraints<<std::endl;

		GraphFramePose* framePoseData = (GraphFramePose*)msg->frameData.data();
        for(unsigned int i=0;i<msg->numFrames;i++)
		{
						std::unordered_map< int, std::shared_ptr<Frame>, std::hash<int>, std::equal_to<int>,
		Eigen::aligned_allocator< std::pair<const int, std::shared_ptr<Frame> > > >::iterator itFrame= keyFrameGraph.at(msg->camId)->idToKeyFrame.find(framePoseData[i].id);
			
			std::unordered_map< int, std::shared_ptr<Frame>, std::hash<int>, std::equal_to<int>,
		Eigen::aligned_allocator< std::pair<const int, std::shared_ptr<Frame> > > >::iterator notYet= keyFrameGraph.at(msg->camId)->idToKeyFrame.end();
				if(itFrame!=notYet)
				{
					std::shared_ptr<Frame> refGraphFrame = keyFrameGraph.at(msg->camId)->idToKeyFrame.find(framePoseData[i].id)->second;
					Frame* graphFrame=refGraphFrame.get(); 
					
					newConstraintMutex.lock();		
					keyFrameGraph.at(camID)->addKeyFrame(graphFrame);
					newConstraintMutex.unlock();
					
					
					keyFrameGraph.at(camID)->keyframesAllMutex.lock_shared();
					keyFrameGraph.at(camID)->keyframesAll.push_back(graphFrame);
					keyFrameGraph.at(camID)->totalVertices ++;
					keyFrameGraph.at(camID)->totalPoints += graphFrame->numPoints;

					keyFrameGraph.at(camID)->keyframesAllMutex.unlock_shared();


					newKeyFrameMutex.lock();
					newKeyFrames.push_back(graphFrame);
					newKeyFrameCreatedSignal.notify_all();
					newKeyFrameMutex.unlock();
					

				}

					

		}




		std::cout<<"Size of newKeyFrames "<<newKeyFrames.size()<<std::endl;
		
		newConstraintMutex.lock();		

		for(unsigned int i=0;i<newKeyFrames.size();i++)
		{
			keyFrameGraph.at(camID)->addKeyFrame(newKeyFrames[i]);
			if (outputWrapper != nullptr)
				//if(!newKF->pose->isInGraph)
					outputWrapper->publishKeyframe(newKeyFrames[i]);
		}
		newConstraintMutex.unlock();
		
			

		
		for(unsigned int i=0;i<numConst;i++)
		{
		
		//	  std::cout<<"Egde From: "<< constraint[i].from <<" To " <<constraint[i].to << " ,Error = "<< constraint[i].err << std::endl;
					
			std::unordered_map< int, std::shared_ptr<Frame>, std::hash<int>, std::equal_to<int>,
		Eigen::aligned_allocator< std::pair<const int, std::shared_ptr<Frame> > > >::iterator itFrom= keyFrameGraph.at(camID)->idToKeyFrame.find(constraint[i].from);

			std::unordered_map< int, std::shared_ptr<Frame>, std::hash<int>, std::equal_to<int>,
		Eigen::aligned_allocator< std::pair<const int, std::shared_ptr<Frame> > > >::iterator itTo= keyFrameGraph.at(camID)->idToKeyFrame.find(constraint[i].to);
			
			std::unordered_map< int, std::shared_ptr<Frame>, std::hash<int>, std::equal_to<int>,
		Eigen::aligned_allocator< std::pair<const int, std::shared_ptr<Frame> > > >::iterator notIn= keyFrameGraph.at(camID)->idToKeyFrame.end();

		
			if((itFrom != notIn )&&(itTo!=notIn))
				{	
				poseConsistencyMutex.lock_shared();
				Constraint.push_back(new KFConstraintStruct());
				reffirst = keyFrameGraph.at(camID)->idToKeyFrame.find(constraint[i].to)->second;
				refsecond = keyFrameGraph.at(camID)->idToKeyFrame.find(constraint[i].from)->second;
				Constraint.back()->firstFrame = reffirst.get();
				Constraint.back()->secondFrame =refsecond.get();
				Constraint.back()->secondToFirst = Constraint.back()->firstFrame->getScaledCamToWorld().inverse() * Constraint.back()->secondFrame->getScaledCamToWorld();
				Constraint.back()->information  <<
						0.8098,-0.1507,-0.0557, 0.1211, 0.7657, 0.0120, 0,
						-0.1507, 2.1724,-0.1103,-1.9279,-0.1182, 0.1943, 0,
						-0.0557,-0.1103, 0.2643,-0.0021,-0.0657,-0.0028, 0.0304,
						 0.1211,-1.9279,-0.0021, 2.3110, 0.1039,-0.0934, 0.0005,
						 0.7657,-0.1182,-0.0657, 0.1039, 1.0545, 0.0743,-0.0028,
						 0.0120, 0.1943,-0.0028,-0.0934, 0.0743, 0.4511, 0,
						0,0, 0.0304, 0.0005,-0.0028, 0, 0.0228;
				Constraint.back()->information *= (1e9/(downweightFac*downweightFac));
				Constraint.back()->robustKernel = new g2o::RobustKernelHuber();
				Constraint.back()->robustKernel->setDelta(kernelDelta);
				Constraint.back()->meanResidual = 10;
				Constraint.back()->meanResidualD = 10;
				Constraint.back()->meanResidualP = 10;
				Constraint.back()->usage = 0;
				poseConsistencyMutex.unlock_shared();
				
				/*newConstraintMutex.lock();		
				keyFrameGraph.at(camID)->addKeyFrame(reffirst.get());
				keyFrameGraph.at(camID)->addKeyFrame(refsecond.get());
				newConstraintMutex.unlock();		
*/



				isIn= true;				
				}
		}

	if(isIn)
	{



		newConstraintMutex.lock();	
		//keyFrameGraph.at(camID)->addKeyFrame(reffirst.get());
		//keyFrameGraph.at(camID)->addKeyFrame(refsecond.get());

		for(unsigned int i=0;i<Constraint.size();i++)
			{
				keyFrameGraph.at(camID)->insertConstraint(Constraint[i]);

			}
		std::cout<<"Num of Constraints received "<<numConst<<std::endl;
		std::cout<<"Num of Constraints in graph "<<Constraint.size()<<std::endl;
		newConstraintAdded = true;
//		Constraint.clear();
		newConstraintCreatedSignal.notify_all();
		newConstraintMutex.unlock();	
		
		//newConstraintMutex.lock();
		//keyFrameGraph.at(camID)->addElementsFromBuffer();
		//newConstraintMutex.unlock();
		
/*		if(newConstraintMutex.try_lock())
	{
	//	std::cout<<"Accesing to Lock 1"<<std::endl;
	//	keyFrameGraph.at(camID)->addElementsFromBuffer();
	//	std::cout<<"Exiting to Lock 1"<<std::endl;
		newConstraintMutex.unlock();
		//newKeyFrames.pop_front();
	}	
*/
 		//TO DO... try to see how to remove outlier frames from graph maybe try to publish keyframes in mapping thread. See whats better buffer in thread 
 		//or in graph callback play with optimization params iterations and min change
		
		
			
		//isIn=false;
		
//		thread_optimization = boost::thread(&MergeSystem::optimizationThread, this);

		//std::cout<<"optimized"<<std::endl;
		
//		isIn=false;				
	}
		
//	}
	/*	newConstraintMutex.lock();
		keyFrameGraph.at(camID)->addElementsFromBuffer();
		newConstraintMutex.unlock();
		*/
		//thread_optimization = boost::thread(&MergeSystem::optimizationThread, this);
		//thread_mapgraph= boost::thread(&MergeSystem::mappingThread, this);

	}
	}
	else
	{
		ROS_INFO("Not ready with prev Graph_msg");
	}
}

// Reads Matching Info 
void MergeSystem::matchCb(coslam_msgs::keyframeMatchInfoConstPtr msg)
{
	if(!doneOpt)
	{
	isIn=false;
	int cam_from;
	int cam_to;
	if(msg->iCam==1)
	{
		cam_from=msg->kfId1;
		cam_to=msg->kfId2;
	}
	else
	{
		cam_from=msg->kfId2;
		cam_to=msg->kfId1;		
	}
	ROS_INFO("Matching Callback");
	std::cout<<"Matching Between Frame "<< msg->kfId1 << " and Frame "<< msg->kfId2 << std::endl;

	Sim3 BtoA_initial;
	memcpy(BtoA_initial.data(), msg->BtoA.data(), sizeof(float)*7);//mencpy copies BtoA into BtoA_initial specyfing the size of data to copy float 7 since XYZ RPY DEPTH

	Constraint.push_back(new KFConstraintStruct());
	std::shared_ptr<Frame> kfId1;
	std::shared_ptr<Frame> kfId2;		

	std::unordered_map< int, std::shared_ptr<Frame>, std::hash<int>, std::equal_to<int>,
Eigen::aligned_allocator< std::pair<const int, std::shared_ptr<Frame> > > >::iterator itKF1= keyFrameGraph.at(camID)->idToKeyFrame.find(cam_from);

	std::unordered_map< int, std::shared_ptr<Frame>, std::hash<int>, std::equal_to<int>,
Eigen::aligned_allocator< std::pair<const int, std::shared_ptr<Frame> > > >::iterator itKF2= keyFrameGraph.at(camID)->idToKeyFrame.find(cam_to);
	
	std::unordered_map< int, std::shared_ptr<Frame>, std::hash<int>, std::equal_to<int>,
Eigen::aligned_allocator< std::pair<const int, std::shared_ptr<Frame> > > >::iterator notIn= keyFrameGraph.at(camID)->idToKeyFrame.end();

		
	if((itKF1 != notIn )&&(itKF2!=notIn))
		{		
			ROS_INFO("ok");
			poseConsistencyMutex.lock_shared();
			constraintMatch.push_back(new KFConstraintStruct());
			kfId1 = keyFrameGraph.at(camID)->idToKeyFrame.find(cam_from)->second;
			kfId2 = keyFrameGraph.at(camID)->idToKeyFrame.find(cam_to)->second;
			constraintMatch.back()->firstFrame = kfId1.get();
			constraintMatch.back()->secondFrame =kfId2.get();
			constraintMatch.back()->secondToFirst = BtoA_initial;//Constraint.back()->firstFrame->getScaledCamToWorld().inverse() * Constraint.back()->secondFrame->getScaledCamToWorld();
			constraintMatch.back()->information  <<
					0.8098,-0.1507,-0.0557, 0.1211, 0.7657, 0.0120, 0,
					-0.1507, 2.1724,-0.1103,-1.9279,-0.1182, 0.1943, 0,
					-0.0557,-0.1103, 0.2643,-0.0021,-0.0657,-0.0028, 0.0304,
					 0.1211,-1.9279,-0.0021, 2.3110, 0.1039,-0.0934, 0.0005,
					 0.7657,-0.1182,-0.0657, 0.1039, 1.0545, 0.0743,-0.0028,
					 0.0120, 0.1943,-0.0028,-0.0934, 0.0743, 0.4511, 0,
					0,0, 0.0304, 0.0005,-0.0028, 0, 0.0228;
			constraintMatch.back()->information *= (1e9/(downweightFac*downweightFac));
			constraintMatch.back()->robustKernel = new g2o::RobustKernelHuber();
			constraintMatch.back()->robustKernel->setDelta(kernelDelta);
			constraintMatch.back()->meanResidual = 10;
			constraintMatch.back()->meanResidualD = 10;
			constraintMatch.back()->meanResidualP = 10;
			constraintMatch.back()->usage = 0;
			poseConsistencyMutex.unlock_shared();

			newConstraintMutex.lock();
			
			poseConsistencyMutex.lock_shared();
			constraintMatch.push_back(new KFConstraintStruct());
			kfId1 = keyFrameGraph.at(camID)->idToKeyFrame.find(cam_to)->second;
			kfId2 = keyFrameGraph.at(camID)->idToKeyFrame.find(cam_from)->second;
			constraintMatch.back()->firstFrame = kfId1.get();
			constraintMatch.back()->secondFrame =kfId2.get();
			constraintMatch.back()->secondToFirst = BtoA_initial.inverse();//Constraint.back()->firstFrame->getScaledCamToWorld().inverse() * Constraint.back()->secondFrame->getScaledCamToWorld();
			constraintMatch.back()->information  <<
					0.8098,-0.1507,-0.0557, 0.1211, 0.7657, 0.0120, 0,
					-0.1507, 2.1724,-0.1103,-1.9279,-0.1182, 0.1943, 0,
					-0.0557,-0.1103, 0.2643,-0.0021,-0.0657,-0.0028, 0.0304,
					 0.1211,-1.9279,-0.0021, 2.3110, 0.1039,-0.0934, 0.0005,
					 0.7657,-0.1182,-0.0657, 0.1039, 1.0545, 0.0743,-0.0028,
					 0.0120, 0.1943,-0.0028,-0.0934, 0.0743, 0.4511, 0,
					0,0, 0.0304, 0.0005,-0.0028, 0, 0.0228;
			constraintMatch.back()->information *= (1e9/(downweightFac*downweightFac));
			constraintMatch.back()->robustKernel = new g2o::RobustKernelHuber();
			constraintMatch.back()->robustKernel->setDelta(kernelDelta);
			constraintMatch.back()->meanResidual = 10;
			constraintMatch.back()->meanResidualD = 10;
			constraintMatch.back()->meanResidualP = 10;
			constraintMatch.back()->usage = 0;
			poseConsistencyMutex.unlock_shared();

			newConstraintMutex.lock();

			for(unsigned int i=0;i<constraintMatch.size();i++)			
				keyFrameGraph.at(camID)->insertConstraint(constraintMatch[i]);
			
			newConstraintAdded = true;
			newConstraintCreatedSignal.notify_all();
			newConstraintMutex.unlock();
			isIn=true;		
			
		}
	}
		
}


   
  
  


// Computes Sim3 Transformation given two frame ids 
void MergeSystem::computeTransSim3(int idA, int idB)
{
 
  // Get pointers for both the frames
  int camIdA = 1;
  int camIdB = 2; 
  std::shared_ptr<Frame> refA = keyFrameGraph.at(camIdA)->idToKeyFrame.find(idA)->second;
  std::shared_ptr<Frame> refB = keyFrameGraph.at(camIdB)->idToKeyFrame.find(idB)->second;
  
  // Construct tracking reference for ref frame
  TrackingReference* trA = new TrackingReference();
  trA->importFrame(refA.get());  
    
  // Get an initial estimate of for the Transformation

  const Sim3 BtoA_initial = refA->getScaledCamToWorld().inverse() * refB->getScaledCamToWorld();
  std::cout<<"Before Optimization :"<< std::endl;
  printSim3(BtoA_initial);
  
  /*
  const Sim3 BtoA_initial_bad = Sim3(); 
  printSim3(BtoA_initial_bad);
  startLvl = 4; endLvl = 3;
  BtoA_final = tracker->trackFrameSim3(trA, refB.get(),BtoA_initial_bad,startLvl, endLvl);
  std::cout<<"After Optimization :"<< std::endl;
  std::cout<<"At Start Level = "<< startLvl << ", End Level = " << endLvl << std::endl;
  printSim3(BtoA_final);
  */
 
  
  // Call track Sim3 function
  Sim3 BtoA_final;
  
  int startLvl = 1;
  int endLvl = 1;
  
  BtoA_final = tracker->trackFrameSim3(trA, refB.get(),BtoA_initial,startLvl, endLvl);
  std::cout<<"After Optimization :"<< std::endl;
  std::cout<<"At Start Level = "<< startLvl << ", End Level = " << endLvl << std::endl;
  printSim3(BtoA_final);
  
  startLvl = 2; endLvl = 2;
  BtoA_final = tracker->trackFrameSim3(trA, refB.get(),BtoA_initial,startLvl, endLvl);
  std::cout<<"After Optimization :"<< std::endl;
  std::cout<<"At Start Level = "<< startLvl << ", End Level = " << endLvl << std::endl;
  printSim3(BtoA_final);
 
  startLvl = 4; endLvl = 3;
  BtoA_final = tracker->trackFrameSim3(trA, refB.get(),BtoA_initial,startLvl, endLvl);
  std::cout<<"After Optimization :"<< std::endl;
  std::cout<<"At Start Level = "<< startLvl << ", End Level = " << endLvl << std::endl;
  printSim3(BtoA_final);
  

  
}



// Debug Information for current Keyframe
void MergeSystem::currentKeyFrame_debug()
{
    // Test compute TransSim3 (Optimization) function
    if(count1==20)
    {
      int camIdA = 1;
      int camIdB = 2;
      int idA = keyFrameGraph.at(camIdA)->keyframesAll[0]->id();
      int idB = keyFrameGraph.at(camIdB)->keyframesAll[9]->id();
      std::cout<<"************************************************************************************"<<std::endl;
      std::cout<<"Computing Sim3 Transformation between : Frame "<< idA << " and " << idB << std::endl;
      computeTransSim3(idA,idB);
      std::cout<<"*************************************************************************************"<<std::endl;
    }
    
    count1 = count1 + 1;
   
  
    // Test Relative pose between consecutive keyframes
    ROS_INFO("Test Relative pose between consecutive keyframes");
    if(count1 == 5)
    {
      Sim3 kf1_pose = keyFrameGraph.at(1)->keyframesAll[count1-1]-> getScaledCamToWorld(); 
      Sim3 kf2_pose = keyFrameGraph.at(2)->keyframesAll[count1]-> getScaledCamToWorld();
      Sim3 kf12_pose = kf1_pose.inverse()*kf2_pose;
      std::cout<<"*********************************************************************************"<<std::endl;
      std::cout<<" Keyframe 1 Pose : "<<std::endl;
      printSim3(kf1_pose);
      std::cout<<" Keyframe 2 Pose : "<<std::endl;
      printSim3(kf2_pose);
      std::cout<<" Keyframe 12 Relative Pose : "<<std::endl;
      printSim3(kf12_pose);
      std::cout<<"*********************************************************************************"<<std::endl;
      
    }
    
  
    // Try to access element from keyframesAll
     /* std::cout<<"KeyframesAll:"<< std::endl; 
      std::cout<<"Keyframe Id = "<<keyFrameGraph->keyframesAll[count1]->id() << std::endl;
      printSim3(keyFrameGraph->keyframesAll[count1]->getScaledCamToWorld());
           
       // Try to access element from idToKeyFrame
      std::cout<<"Keyframes idToKeyFrame:"<< std::endl; 
      std::cout<<"Keyframe Id = "<< keyFrameGraph->idToKeyFrame.find(keyFrameGraph->keyframesAll[count1]->id())->second->id()<< std::endl;
      //std::cout<<"Keyframe Id = "<< keyFrameGraph->idToKeyFrame.find(keyFrameGraph->keyframesAll[count1]->id())->second->getScaledCamToWorld()<< std::endl;
      count1 = count1 + 1;
      
      // Test Initialising a sim3 from vector
      
      /*std::vector<float> c2w_vec(7);
      c2w_vec.at(0) = 1; c2w_vec.at(1) = 2; c2w_vec.at(2) = 3; c2w_vec.at(3) = 4;
      c2w_vec.at(4) = 5; c2w_vec.at(5) = 6; c2w_vec.at(6) = 7;
      Sim3 c2w;
      memcpy(c2w.data(), c2w_vec.data(), sizeof(float)*7);
      printSim3(c2w);
      */
      
     /* IDepth Var
     if (debug_count == 1)
     {
       std::cout<<"Image Depth Variance" << idepth << std::endl;
       
     }
     */
     
     /* // Compute mean depth of frame
     float m_inv_depth = 0.0;
     int num_idepth_points = 0;
     for(int i =0; i<h; i++) // For each row
     {
        float* data_idepth = idepth.ptr<float>(i);
             
	for(int j = 0; j<w ; j++) // for each column
	{
	  if(data_idepth[j]>0)
	  { m_inv_depth += data_idepth[j];
	    num_idepth_points += 1;
	  }
	}
     }
     m_inv_depth = m_inv_depth/(float)num_idepth_points;
     std::cout<<"Mean Image Depth = " << m_inv_depth<< std::endl;
     */
     
     /* // Keyframe Image Debug
     int lvl = 2;
     cv::Mat image_debug = cv::Mat(currentKeyFrame->height(lvl), currentKeyFrame->width(lvl),CV_8U);
     const float* pt1 = currentKeyFrame->image(lvl);
     for(int i =0 ; i< currentKeyFrame->height(lvl) ; i++) // For each row
     {
        uchar* data_image_debug = image_debug.ptr<uchar>(i);
             
	for(int j = 0; j < currentKeyFrame->width(lvl) ; j++) // for each column
	{  
	  data_image_debug[j] = (uchar)*pt1; 
	  pt1++;
	}
     }

     cv::imshow("Image debug Frame",image_debug);
     cv::waitKey(5);
     */ 
     
    /* // Keyframe IDepth/Var Debug Info
    // To correct : The idepth set in frame doesnt look like the one we read from keyframe message
  
     cv::Mat idepth_debug = cv::Mat(currentKeyFrame->height(lvl), currentKeyFrame->width(lvl),CV_32F);
     const float* pt2 = currentKeyFrame->idepth(lvl);
     for(int i =0 ; i< currentKeyFrame->height(lvl) ; i++) // For each row
     {
        float* data_idepth_debug = idepth_debug.ptr<float>(i);
             
	for(int j = 0; j < currentKeyFrame->width(lvl) ; j++) // for each column
	{  
	  data_idepth_debug[j] = *pt2; 
	  pt2++;
	}
     }

     cv::imshow("IDepth Debug Frame",idepth_debug);
     cv::waitKey(5);
     */
    
    // Keyframe Graph Info
    //std::cout<< "Keyframe Graph Size =" << keyFrameGraph->keyframesAll.size() << std::endl;

  
}


void MergeSystem::printSim3(Sim3 pose)
{
  //std::cout<<"Pose Quaternion ="<<pose.quaternion().w()<<","<<pose.quaternion().x()<< "," << pose.quaternion().y() <<"," << pose.quaternion().z() <<std::endl;
  std::cout<<"Rotation Matrix ="<<pose.rotationMatrix()<<std::endl;
  std::cout<<"Pose Translation ="<<pose.translation()[0]<<","<< pose.translation()[1] <<","<< pose.translation()[2] <<std::endl;
  std::cout<<"Scale ="<<pose.scale() << std::endl;
 
}



//Only stores the values of the constraints and graph to later do the optimization and graph building
/*void MergeSystem::graphCb1Debug(lsd_slam_viewer::keyframeGraphMsgConstPtr msg)
{
		ROS_INFO("Graph Callback");

	  //KeyFrame Pose Related 

	 framePoseData = (GraphFramePose*)msg->frameData.data();
	 camID=msg->camId;
	 numConst=msg->numConstraints;

	  for(unsigned int i = 0; i < msg->numFrames; i++)
	  {
		std::vector<double> camToWorld(7);
		for(unsigned int i = 0; i < 7;  i++)
		  {   
			  camToWorld[i]  = (double) framePoseData[i].camToWorld[i];
		  }
		memcpy(poseKeyFrame.data(),camToWorld.data(), sizeof(double)*7);
	  }  

		 constraint.push_back(msg->constraintsData.data());
	
}

*/

/*void MergeSystem::buildGraph()
{
	ROS_INFO("Graph Callback");
	this_time = clock();


	  
	  bool isIn= false;
	  bool	needToPublish= false;

	//Edges Constraint Related
	
	for(unsigned int i=0;i<numConst; i++)
	{	
		//  std::cout<<"Egde From: "<< constraint[i].from <<" To " <<constraint[i].to << " ,Error = "<< constraint[i].err << std::endl;
				
		std::unordered_map< int, std::shared_ptr<Frame>, std::hash<int>, std::equal_to<int>,
	Eigen::aligned_allocator< std::pair<const int, std::shared_ptr<Frame> > > >::iterator itFrom= keyFrameGraph.at(camID)->idToKeyFrame.find(constraint[i].from);

		std::unordered_map< int, std::shared_ptr<Frame>, std::hash<int>, std::equal_to<int>,
	Eigen::aligned_allocator< std::pair<const int, std::shared_ptr<Frame> > > >::iterator itTo= keyFrameGraph.at(camID)->idToKeyFrame.find(constraint[i].to);
		
		std::unordered_map< int, std::shared_ptr<Frame>, std::hash<int>, std::equal_to<int>,
	Eigen::aligned_allocator< std::pair<const int, std::shared_ptr<Frame> > > >::iterator notIn= keyFrameGraph.at(camID)->idToKeyFrame.end();

	
		if((itFrom != notIn )&&(itTo!=notIn))
				{	
			poseConsistencyMutex.lock_shared();
			Constraint.push_back(new KFConstraintStruct());
		//	ROS_INFO("ok");
			std::shared_ptr<Frame> reffirst = keyFrameGraph.at(camID)->idToKeyFrame.find(constraint[i].from)->second;
			std::shared_ptr<Frame> refsecond = keyFrameGraph.at(camID)->idToKeyFrame.find(constraint[i].to)->second;
			Constraint.back()->firstFrame = reffirst.get();
			Constraint.back()->secondFrame =refsecond.get();
			Constraint.back()->secondToFirst = Constraint.back()->firstFrame->getScaledCamToWorld().inverse() * Constraint.back()->secondFrame->getScaledCamToWorld();
			Constraint.back()->information  <<
					0.8098,-0.1507,-0.0557, 0.1211, 0.7657, 0.0120, 0,
					-0.1507, 2.1724,-0.1103,-1.9279,-0.1182, 0.1943, 0,
					-0.0557,-0.1103, 0.2643,-0.0021,-0.0657,-0.0028, 0.0304,
					 0.1211,-1.9279,-0.0021, 2.3110, 0.1039,-0.0934, 0.0005,
					 0.7657,-0.1182,-0.0657, 0.1039, 1.0545, 0.0743,-0.0028,
					 0.0120, 0.1943,-0.0028,-0.0934, 0.0743, 0.4511, 0,
					0,0, 0.0304, 0.0005,-0.0028, 0, 0.0228;
			Constraint.back()->information *= (1e9/(downweightFac*downweightFac));
			Constraint.back()->robustKernel = new g2o::RobustKernelHuber();
			Constraint.back()->robustKernel->setDelta(kernelDelta);
			Constraint.back()->meanResidual = 10;
			Constraint.back()->meanResidualD = 10;
			Constraint.back()->meanResidualP = 10;
			Constraint.back()->usage = 0;
			poseConsistencyMutex.unlock_shared();
			isIn= true;
		//}
		}
		}
	if(isIn)
	{
		newConstraintMutex.lock();
		for(unsigned int i=0;i<Constraint.size();i++)
			{
				keyFrameGraph.at(camID)->insertConstraint(Constraint[i]);
			}
		newConstraintMutex.unlock();
		newConstraintAdded = true;
		newConstraintCreatedSignal.notify_all();
		newConstraintMutex.unlock();
		std::cout<<"optimizing"<<std::endl;
		keyFrameGraph.at(camID)->optimize(1000);
		
		
		needToPublish=true;
				
					}
		
	//}
	
	
	
	
    if (needToPublish)
		{   
		outputWrapper->publishKeyframeGraph(keyFrameGraph.at(camID));	
	//	std::cout<<"Publishing graph with num constraints:"<<keyFrameGraph.at(camID)->edgesAll.size()<<std::endl;
		}	
		
		
	
}


*/
   




