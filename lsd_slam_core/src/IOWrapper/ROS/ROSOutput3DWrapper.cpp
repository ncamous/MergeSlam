/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ROSOutput3DWrapper.h"
#include "util/SophusUtil.h"
#include <ros/ros.h>
#include "util/settings.h"


#include "std_msgs/Float32MultiArray.h"
#include "lsd_slam_viewer/keyframeGraphMsg.h"
#include "lsd_slam_viewer/keyframeMsg.h"
//#include "place_recognizer/keyframeImg.h"

#include "DataStructures/Frame.h"
#include "GlobalMapping/KeyFrameGraph.h"
#include "sophus/sim3.hpp"
#include "geometry_msgs/PoseStamped.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h>
#include "GlobalMapping/g2oTypeSim3Sophus.h"

namespace lsd_slam
{


ROSOutput3DWrapper::ROSOutput3DWrapper(int width, int height)
{
	this->width = width;
	this->height = height;

	liveframe_channel = nh_.resolveName("lsd_slam/liveframes");
	liveframe_publisher = nh_.advertise<lsd_slam_viewer::keyframeMsg>(liveframe_channel,1);

	keyframe_channel = nh_.resolveName("lsd_slam/keyframes");
	keyframe_publisher = nh_.advertise<lsd_slam_viewer::keyframeMsg>(keyframe_channel,1);

	//keyframeImg_channel = nh_.resolveName("lsd_slam/keyframeImgs");
	//keyframeImg_publisher = nh_.advertise<sensor_msgs::Image>(keyframeImg_channel,1);
	
	//keyframeImg_channel = nh_.resolveName("lsd_slam/keyframeImgs");
	//keyframeImg_publisher = nh_.advertise<place_recognizer::keyframeImg>(keyframeImg_channel,1);

	graph_channel = nh_.resolveName("lsd_slam/graph");
	graph_publisher = nh_.advertise<lsd_slam_viewer::keyframeGraphMsg>(graph_channel,1);

	debugInfo_channel = nh_.resolveName("lsd_slam/debug");
	debugInfo_publisher = nh_.advertise<std_msgs::Float32MultiArray>(debugInfo_channel,1);

	pose_channel = nh_.resolveName("lsd_slam/pose");
	pose_publisher = nh_.advertise<geometry_msgs::PoseStamped>(pose_channel,1);


	publishLvl=0;
}

ROSOutput3DWrapper::ROSOutput3DWrapper(int width, int height, int camId)
{
	this->width = width;
	this->height = height;
	this->camId = camId;
	
	liveframe_channel = nh_.resolveName("lsd_slam/liveframes");
	liveframe_publisher = nh_.advertise<lsd_slam_viewer::keyframeMsg>(liveframe_channel,1);

	keyframe_channel = nh_.resolveName("lsd_slam/keyframes");
	keyframe_publisher = nh_.advertise<lsd_slam_viewer::keyframeMsg>(keyframe_channel,1);

	//keyframeImg_channel = nh_.resolveName("lsd_slam/keyframeImgs");
	//keyframeImg_publisher = nh_.advertise<sensor_msgs::Image>(keyframeImg_channel,1);
	
	//keyframeImg_channel = nh_.resolveName("lsd_slam/keyframeImgs");
	//keyframeImg_publisher = nh_.advertise<place_recognizer::keyframeImg>(keyframeImg_channel,1);

	graph_channel = nh_.resolveName("lsd_slam/graph");
	graph_publisher = nh_.advertise<lsd_slam_viewer::keyframeGraphMsg>(graph_channel,1);

	debugInfo_channel = nh_.resolveName("lsd_slam/debug");
	debugInfo_publisher = nh_.advertise<std_msgs::Float32MultiArray>(debugInfo_channel,1);

	pose_channel = nh_.resolveName("lsd_slam/pose");
	pose_publisher = nh_.advertise<geometry_msgs::PoseStamped>(pose_channel,1);


	publishLvl=0;
}

ROSOutput3DWrapper::~ROSOutput3DWrapper()
{
}


void ROSOutput3DWrapper::publishKeyframe(Frame* f)
{
	lsd_slam_viewer::keyframeMsg fMsg;


	boost::shared_lock<boost::shared_mutex> lock = f->getActiveLock();

	fMsg.camId = camId;
	fMsg.id = f->id();
	fMsg.time = f->timestamp();
	fMsg.isKeyframe = true;

	int w = f->width(publishLvl);
	int h = f->height(publishLvl);
	
	// For Debug Purposes *ADDED
	std::cout<<"Keyframe Id = "<< fMsg.id << std::endl;
	std::cout<<"Pose Quaternion ="<<f->getScaledCamToWorld().quaternion().w()<<","<<f->getScaledCamToWorld().quaternion().x()<< "," << f->getScaledCamToWorld().quaternion().y() <<"," << f->getScaledCamToWorld().quaternion().z() <<std::endl;
	std::cout<<"Pose Translation ="<<f->getScaledCamToWorld().translation()[0]<<","<<f->getScaledCamToWorld().translation()[1] <<","<< f->getScaledCamToWorld().translation()[2] <<std::endl;
	std::cout<<"Scale ="<<f->getScaledCamToWorld().scale() << std::endl;
	//std::cout<<"RxSO3 ="<<f->getScaledCamToWorld().rxso3() << std::endl;
	std::cout<<"Translation ="<<f->getScaledCamToWorld().translation() << std::endl;
	
//
	memcpy(fMsg.camToWorld.data(),f->getScaledCamToWorld().cast<float>().data(),sizeof(float)*7); //destination, source, size of data casted to 7 decimals
	
	// Debug After Copying
/*	for(unsigned int i = 0; i < fMsg.camToWorld.size();i++)
	      std::cout<< fMsg.camToWorld[i]<<",";
	std::cout<<std::endl;
*/	
	fMsg.fx = f->fx(publishLvl);
	fMsg.fy = f->fy(publishLvl);
	fMsg.cx = f->cx(publishLvl);
	fMsg.cy = f->cy(publishLvl);
	fMsg.width = w;
	fMsg.height = h;


	fMsg.pointcloud.resize(w*h*sizeof(InputPointDense));

	InputPointDense* pc = (InputPointDense*)fMsg.pointcloud.data();

	const float* idepth = f->idepth(publishLvl);
	const float* idepthVar = f->idepthVar(publishLvl);
	const float* color = f->image(publishLvl);

	for(int idx=0;idx < w*h; idx++)
	{
		pc[idx].idepth = idepth[idx];
		pc[idx].idepth_var = idepthVar[idx];
		pc[idx].color[0] = color[idx];
		pc[idx].color[1] = color[idx];
		pc[idx].color[2] = color[idx];
		pc[idx].color[3] = color[idx];
	}

	keyframe_publisher.publish(fMsg);
}

//void ROSOutput3DWrapper::publishKeyframeImg(Frame* f)
//{ 	
	/*
	// Convert image to ROS sensor_msgs/Image and publish
	//Add id num and other meta information
	cv_bridge::CvImage tmp_img;
	boost::shared_lock<boost::shared_mutex> lock = f->getActiveLock();
	cv::Mat kfImgCV(f->height(), f->width(), CV_32F, const_cast<float*>(f->image()));
	kfImgCV.convertTo(kfImgCV, CV_8UC1);
	// Display image in openCV window (For debugging)
	//Util::displayImage("Keyframe Image OpenCV", const_cast<float*>(f->image()), f->width(), f->height());
	tmp_img.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
	tmp_img.image    = kfImgCV; // Your cv::Mat
	keyframeImg_publisher.publish(tmp_img.toImageMsg()); */
	
	

	/*
	// Convert image to ROS sensor_msgs/Image, add image id etc and publish
	cv_bridge::CvImage tmp_img;
	sensor_msgs::Image fImg;
	place_recognizer::keyframeImg fMsg;
	boost::shared_lock<boost::shared_mutex> lock = f->getActiveLock();
	
	//camera parameters
	int w = f->width(publishLvl);
	int h = f->height(publishLvl);
	fMsg.fx = f->fx(publishLvl);
	fMsg.fy = f->fy(publishLvl);
	fMsg.cx = f->cx(publishLvl);
	fMsg.cy = f->cy(publishLvl);
	fMsg.width = w;
	fMsg.height = h;
	
	// Extract image from frame structure and covert into sensor_msgs/Image format
	cv::Mat kfImgCV(f->height(), f->width(), CV_32F, const_cast<float*>(f->image()));
	kfImgCV.convertTo(kfImgCV, CV_8UC1);
	tmp_img.encoding = sensor_msgs::image_encodings::MONO8;
	tmp_img.image    = kfImgCV;
	tmp_img.toImageMsg(fImg);
	
	// Extract depth map from frame structure and convert into sensor_msgs/Image format
	cv_bridge::CvImage tmp_dep;
	sensor_msgs::Image fDep;
	cv::Mat kfDepCV(f->height(), f->width(), CV_32F, const_cast<float*>(f->idepth()));
	tmp_dep.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
	tmp_dep.image = kfDepCV;
	tmp_dep.toImageMsg(fDep);
	
	//Copy image id , timestamp and keyframe image to msg and publish  
	fMsg.id = f->id();
	fMsg.time = f->timestamp();
	fMsg.img = fImg; 
	fMsg.idep = fDep; 
	
	// Publish keyframeImg message
	keyframeImg_publisher.publish(fMsg); 
 
}
*/

void ROSOutput3DWrapper::publishTrackedFrame(Frame* kf)
{
	lsd_slam_viewer::keyframeMsg fMsg;
	fMsg.camId = camId;
	fMsg.id = kf->id();
	fMsg.time = kf->timestamp();
	fMsg.isKeyframe = false;


	memcpy(fMsg.camToWorld.data(),kf->getScaledCamToWorld().cast<float>().data(),sizeof(float)*7);
	fMsg.fx = kf->fx(publishLvl);
	fMsg.fy = kf->fy(publishLvl);
	fMsg.cx = kf->cx(publishLvl);
	fMsg.cy = kf->cy(publishLvl);
	fMsg.width = kf->width(publishLvl);
	fMsg.height = kf->height(publishLvl);

	fMsg.pointcloud.clear();

	liveframe_publisher.publish(fMsg);


	SE3 camToWorld = se3FromSim3(kf->getScaledCamToWorld());

	geometry_msgs::PoseStamped pMsg;

	pMsg.pose.position.x = camToWorld.translation()[0];
	pMsg.pose.position.y = camToWorld.translation()[1];
	pMsg.pose.position.z = camToWorld.translation()[2];
	pMsg.pose.orientation.x = camToWorld.so3().unit_quaternion().x();
	pMsg.pose.orientation.y = camToWorld.so3().unit_quaternion().y();
	pMsg.pose.orientation.z = camToWorld.so3().unit_quaternion().z();
	pMsg.pose.orientation.w = camToWorld.so3().unit_quaternion().w();

	if (pMsg.pose.orientation.w < 0)
	{
		pMsg.pose.orientation.x *= -1;
		pMsg.pose.orientation.y *= -1;
		pMsg.pose.orientation.z *= -1;
		pMsg.pose.orientation.w *= -1;
	}

	pMsg.header.stamp = ros::Time(kf->timestamp());
	pMsg.header.frame_id = "world";
	pose_publisher.publish(pMsg);
}




void ROSOutput3DWrapper::publishKeyframeGraph(KeyFrameGraph* graph)
{

	lsd_slam_viewer::keyframeGraphMsg gMsg;
	gMsg.camId = camId;
	
	graph->edgesListsMutex.lock();
	gMsg.numConstraints = graph->edgesAll.size();
	gMsg.constraintsData.resize(gMsg.numConstraints * sizeof(GraphConstraint));
	GraphConstraint* constraintData = (GraphConstraint*)gMsg.constraintsData.data();

	for(unsigned int i=0;i<graph->edgesAll.size();i++)
	{
		constraintData[i].from = graph->edgesAll[i]->firstFrame->id();
		constraintData[i].to = graph->edgesAll[i]->secondFrame->id();
		Sophus::Vector7d err = graph->edgesAll[i]->edge->error();
		constraintData[i].err = sqrt(err.dot(err));
	}
	
	graph->edgesListsMutex.unlock();
//	std::cout<<"Edges "<<std::endl;

//Exception of sophus negative scale is trown in this section

	//std::cout<<"Mutex locked"<<std::endl;
	graph->keyframesAllMutex.lock_shared();
	gMsg.numFrames = graph->keyframesAll.size();
	gMsg.frameData.resize(gMsg.numFrames * sizeof(GraphFramePose));
	//std::cout<<"Resizing"<<std::endl;

	GraphFramePose* framePoseData = (GraphFramePose*)gMsg.frameData.data();
	for(unsigned int i=0;i<graph->keyframesAll.size();i++)
	{
		framePoseData[i].id = graph->keyframesAll[i]->id();
		memcpy(framePoseData[i].camToWorld, graph->keyframesAll[i]->getScaledCamToWorld().cast<float>().data(),sizeof(float)*7);
		//std::cout<<"Frame pose "<< i<<std::endl;
	}
//	std::cout<<"Frame pose stored unlock"<<std::endl;
	graph->keyframesAllMutex.unlock_shared();

	//std::cout<<"Frame Pose pub"<<std::endl;


	graph_publisher.publish(gMsg);
}

void ROSOutput3DWrapper::publishTrajectory(std::vector<Eigen::Matrix<float, 3, 1>> trajectory, std::string identifier)
{
	// unimplemented ... do i need it?
}

void ROSOutput3DWrapper::publishTrajectoryIncrement(Eigen::Matrix<float, 3, 1> pt, std::string identifier)
{
	// unimplemented ... do i need it?
}

void ROSOutput3DWrapper::publishDebugInfo(Eigen::Matrix<float, 20, 1> data)
{
	std_msgs::Float32MultiArray msg;
	for(int i=0;i<20;i++)
		msg.data.push_back((float)(data[i]));

	debugInfo_publisher.publish(msg);
}

}
