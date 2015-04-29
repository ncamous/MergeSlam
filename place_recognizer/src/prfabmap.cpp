#include "prfabmap.h"
#include "Sim3dSolver.h"

PRfabmap::PRfabmap()
{ 
  // set subscribers
  sub_kf1 = nh.subscribe("keyframeImgs1", 10, &PRfabmap::kfCb1, this);
  sub_kf2 = nh.subscribe("keyframeImgs2", 10, &PRfabmap::kfCb2, this);
  
  // set Publishers
  pub_match = nh.advertise<place_recognizer::keyframeMatchInfo>("matchInfo",1);
  valid = false; // Make true at the end of the constructor
  //static const std::string basePath = "/home/alphabot/stage_ws/src/place_recognizer/"; // Base path (Change to relative path)
  basePath = ros::package::getPath("place_recognizer")+"/"; 
  
  //Load vocabulary, training data and Chow-Liu tree path
  std::string vocabPath = (basePath + "data/StLuciaShortVocabulary.yml").c_str();
  std::string fabmapTrainDataPath = (basePath + "data/StLuciaShortTraindata.yml").c_str();
  std::string chowliutreePath = (basePath + "data/StLuciaShortTree.yml").c_str();

  // Load vocabulary
  cv::FileStorage fsVocabulary;
  fsVocabulary.open(vocabPath, cv::FileStorage::READ);
  cv::Mat vocabulary;
  fsVocabulary["Vocabulary"] >> vocabulary;
  if (vocabulary.empty())
  {
    std::cerr << vocabPath << ": Vocabulary not found" << std::endl;
    return;
  }
  fsVocabulary.release();
 
  // Load training data
  cv::FileStorage fsTraining;
  fsTraining.open(fabmapTrainDataPath, cv::FileStorage::READ);
  cv::Mat fabmapTrainData;
  fsTraining["BOWImageDescs"] >> fabmapTrainData;
  if (fabmapTrainData.empty()) {
    std::cerr << fabmapTrainDataPath << ": FabMap Training Data not found" << std::endl;
    return;
  }
  fsTraining.release();
  
  //Load Chow-Liu tree path
  cv::FileStorage fsTree;
  fsTree.open(chowliutreePath, cv::FileStorage::READ);
  cv::Mat clTree;
  fsTree["ChowLiuTree"] >> clTree;
  if (clTree.empty()) {
    std::cerr << chowliutreePath << ": Chow-Liu tree not found" << std::endl;
    return;
  }
  fsTree.release();


  // Generate openFabMap object (FabMap2 - needs training bow data!)
  int options = 0;
  options |= cv::of2::FabMap::SAMPLED;
  options |= cv::of2::FabMap::CHOW_LIU;
  fabMap = new cv::of2::FabMap2(clTree, 0.39, 0, options);
  fabMap->addTraining(fabmapTrainData); //add the training data for use with the sampling method
  
  // Create detector & extractor
  int minHessian = 400;
  detector =  new cv::SurfFeatureDetector( minHessian );
  //detector = new cv::StarFeatureDetector(32, 10, 18, 18, 20); // alternative detector
 
  extractor = new cv::SURF(1000, 4, 2, false, true); // alternative:  cv::SIFT();
  //use a FLANN matcher to generate bag-of-words representations
  matcher = cv::DescriptorMatcher::create("FlannBased"); // alternative: "BruteForce"
  bide = new cv::BOWImgDescriptorExtractor(extractor, matcher);
  bide->setVocabulary(vocabulary);

  //Initialize confusion Matrix
  // confusionMat = cv::Mat(0, 0, CV_32F);

  // Set Matching threshold
  matchThresh = 0.8;
  
  // Fabmap initialized properly
  valid = true; 
  
}


// Need to change this 
PRfabmap::~PRfabmap()
{
  std::cout<<"Exiting PRfabmap"<<std::endl;
  
  //For Debugging
  //static const std::string basePath = "/home/alphabot/stage_ws/src/place_recognizer/"; // Base path
  
  std::ofstream writer(( basePath + "/fabMapResult.txt").c_str());
  for(int i = 0; i < confusionMat.rows; i++) {
     for(int j = 0; j < confusionMat.cols; j++) {
	 writer << confusionMat.at<float>(i, j) << " ";
	}
  	 writer << std::endl;
  }
  writer.close();
 
}

void PRfabmap::addNewKeyframeBOW(place_recognizer::keyframeImg& kf_msg, int camId)
{
  // Convert image to cv::Mat using cv_bridge
  cv_bridge::CvImagePtr cv_ptr; 
  try
  {
    cv_ptr = cv_bridge::toCvCopy(kf_msg.img, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat frame;
  cv_ptr->image.copyTo(frame);
  
  
  // Convert idepth to cv::Mat using cv_bridge
  cv_bridge::CvImagePtr dep_ptr;
  try
  {
    dep_ptr = cv_bridge::toCvCopy(kf_msg.idep, sensor_msgs::image_encodings::TYPE_32FC1); 
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception %s", e.what());
  }
  cv::Mat idepth;
  dep_ptr->image.copyTo(idepth);
  
  // Debug Info:
  /*
  std::cout<<"idepth size"<< idepth.size() << "Type = "<< idepth.type() << std::endl;
  double minVal, maxVal;
  cv::minMaxLoc( idepth, &minVal, &maxVal );
  std::cout << "minVal = " << minVal << std::endl;
  std::cout << "maxVal = " << maxVal << std::endl;
  */
  
  // Compute BOW 
  // 1) Detect Features (parameters in the constructor)
  std::vector<cv::KeyPoint> kpts;
  detector->detect(frame, kpts);
     
  // Compute good features (Add here)
  
  // 2) Compute BOW description of the image
  // Temporary implementation for two cameras. (Replace with vector later)
  
  cv::Mat bow;
  if(camId==1){
    
    
    bide->compute(frame, kpts, bow);
    bow1.push_back(bow);
    fId1.push_back(kf_msg.id);
    
    // Store keypoints from each frame
    kptsVec1.push_back(kpts);
  
    // Compute Image Descriptors 
    
    
    
    // Get 2d feature locations and store them 
    cv::Mat p2dC1;
    computeKeyPointLocs(kpts,p2dC1);   
    X2dC1.push_back(p2dC1);
      
    
    // Get 3d feature locations and store them    
    cv::Mat p3dC1;
    computeKeyPoint3d(idepth, p2dC1, p3dC1, camId); // Replace frame by depth map
    X3dC1.push_back(p3dC1);
    
    // Compare with keyframes from camera 2
    if(!bow2.empty())
      compareKeyframeBOW(bow, camId);
    
    }
  else if(camId==2){
    bide->compute(frame, kpts, bow);
    bow2.push_back(bow);
    fId2.push_back(kf_msg.id);
    
    // Store keypoints from each frame
    kptsVec2.push_back(kpts);
    
    // Get 2d feature locations and store them 
    cv::Mat p2dC2;
    computeKeyPointLocs(kpts,p2dC2);   
    X2dC2.push_back(p2dC2);
      
    
    // Get 3d feature locations and store them    
    cv::Mat p3dC2;
    computeKeyPoint3d(idepth, p2dC2, p3dC2, camId); // Replace frame by depth map
    X3dC2.push_back(p3dC2);
    
    
    // Compare with keyframes from camera 1
    if(!bow1.empty())
      compareKeyframeBOW(bow,camId);
    }
    
  // std::cout<<"Size of X3dC1 = "<< X3dC1.size()<<std::endl;
  // std::cout<<"Size of X3dC2 = "<< X3dC2.size()<<std::endl;  
    
  // For Debugging : Visualize keypoints
  /* 
  static const std::string FEATURES_DEBUG_WINDOW1 = "Cam 1";
  static const std::string FEATURES_DEBUG_WINDOW2 = "Cam 2";
  if(camId==1){
  cv::drawKeypoints(frame, kpts, frame);
  cv::imshow(FEATURES_DEBUG_WINDOW1, frame);
  cv::waitKey(1);}
  else{
  cv::drawKeypoints(frame, kpts, frame);
  cv::imshow(FEATURES_DEBUG_WINDOW2, frame);
  cv::waitKey(1);} 
  */

}

void PRfabmap::computeKeyPointLocs(std::vector<cv::KeyPoint>& kpts, cv::Mat& p2d)
{ 
  int nPts = kpts.size();
  
  p2d.create(2,nPts,CV_32F);
  for(int i = 0; i < nPts; i++)
  {
     p2d.at<float>(0,i) = kpts[i].pt.x;
     p2d.at<float>(1,i) = kpts[i].pt.y;
  }
  
}

void PRfabmap::computeKeyPoint3d(cv::Mat& dMap, cv::Mat& p2d,cv::Mat& p3d, int camId)
{
  float f;
  float cX;
  float cY;
  int u = 0;
  int v = 0;
  float idep = 0.0;
  float X,Y,Z;
 
  // Compute 3d point here using camera calibration parameters
  if(camId == 1)
  {
    // Camera 1 Parameters
    f = fx1;   // Focal length
    cX = cx1;  // Center X
    cY = cy1;  // Center Y
    
  }
  else if(camId == 2)
  {
    // Camera 2 Parameters
    f = fx2;   // Focal length
    cX = cx2;  // Center X
    cY = cy2;  // Center Y
   
  }
  
  int nPts = p2d.cols;
  
  p3d.create(3,nPts,CV_32F);
  for(int i = 0; i< p2d.cols; i++)
  { 
    u =  static_cast<int>(p2d.at<float>(1,i));
    v =  static_cast<int>(p2d.at<float>(0,i));
    idep =  dMap.at<float>(u,v); 
			  
    Z = 1/idep;
    X =  ((u-cX)*Z)/f;
    Y =  ((v-cY)*Z)/f;
    
   p3d.at<float>(0,i) = X;
   p3d.at<float>(1,i) = Y;
   p3d.at<float>(2,i) = Z;
   
    
  }
  
 
}


void PRfabmap::compareKeyframeBOW(cv::Mat bow, int camId)
{ 
  // Run FabMap
  std::vector<cv::of2::IMatch> matches;
  std::vector<double> result;
  int qId = -1; 
  
  if(camId==1){
      qId = bow1.rows-1;
      std::cout<<"Cam 1: Compare "<<qId<<" with "<<" Cam 2"<<std::endl;
      fabMap->compare(bow, bow2, matches);  // compare(query_img, test_img, match)
     }    
  else if (camId==2){
      qId = bow2.rows-1;
      std::cout<<"Cam 2: Compare "<<qId<<" with "<<"Cam 1"<<std::endl;
      fabMap->compare(bow, bow1, matches);  // compare(query_img, test_img, match)
  }  
  
  // Store results and publish them
  std::vector<cv::of2::IMatch>::iterator l;
  for(l= matches.begin(); l!=matches.end(); l++)
  {   
      if(l->imgIdx != -1)
	result.push_back(l->match);   // Probability of matching
  }
 

 place_recognizer::keyframeMatchInfo msg;
  
  for(int j=0; j<result.size() ;j++){
     if(result[j] > matchThresh) 
      { 
	std::cout<<"Candidate Match Found"<<std::endl;
	 // Publish match info
	 msg.isMatch = true;
	 msg.matchProb = result[j];
	 msg.iCam = camId;
	 
	 if(camId == 1){
	  msg.kfId1 = qId; //From Cam 1 
	  msg.kfId2 = j;//fId2[j]; //From Cam 2
	 }
	 else if(camId == 2){
	  msg.kfId1 = j;// fId1[j];
	  msg.kfId2 = qId; 
	 }
	 
	 pub_match.publish(msg);
	 
	 
	 
	 
	 // Compute 3d Sim Transformation
	 if(camId==1){
	  
	   
	  // Get 2D feature location matches
	   
	  // Get 3D Points for matching features  
	   
	  // Compute Sim3 Transformation
	  // Some debug info 
	  cv::Mat tmp1 = cv::Mat(X3dC1[qId]);
	  cv::Mat tmp2 = cv::Mat(X3dC2[j]);
	  std::cout<<"Pointcloud 1"<<tmp1.size()<<std::endl<<"Pointcloud 2"<<tmp2.size()<<std::endl;
	   
	  // Solve for sim3 transformation 
	  Sim3dSolver sim(tmp1,tmp2);
	  cv::Mat T12 = sim.solve();
	  std::cout<<"T12"<<"Between frame:"<<qId<<","<<j<<std::endl<<T12<<std::endl;
	 }
	 
	 else if(camId==2){
	  
	  // Similar to cam 1 
	   
	   
	 }
	 
      }
    }
 
}

void PRfabmap::publishMatchInfo(cv::Mat matchMat, int camId, int fId)
{
  std::cout<<"publishMatchInfo"<<std::endl;  
  place_recognizer::keyframeMatchInfo msg;
  
  for(int j=0; j< matchMat.cols ;j++){
     if(matchMat.at<float>(1,j) > matchThresh) 
      { 
	std::cout<<"Candidate Match Found"<<std::endl;
	 // Publish match info
	 msg.isMatch = true;
	 msg.matchProb = matchMat.at<float>(1,j);
	  
	 if(camId == 1){
	  msg.kfId1 = fId; //From Cam 1 
	  msg.kfId2 = fId;//fId2[j]; //From Cam 2
	 }
	 else if(camId == 2){
	  msg.kfId1 = fId;// fId1[j];
	  msg.kfId2 = fId; 
	 }
	 
	 pub_match.publish(msg);
	
      }
    }
  
}

void PRfabmap::kfCb1(const place_recognizer::keyframeImg::ConstPtr& fmsg)
{
  int camId = 1;
  ROS_INFO("Image Callback Function 1 : Reading Image Number = %d", fmsg->id);
  
  // Get camera parameters
  fx1 = fmsg->fx;
  fy1 = fmsg->fy;
  cx1 = fmsg->cx;
  cy1 = fmsg->cy;
  height1 = fmsg->height;
  width1 = fmsg->width;
  
  // Add new keyframe
  place_recognizer::keyframeImg kf_msg;
  kf_msg = *fmsg;
  addNewKeyframeBOW(kf_msg, camId);
  
 
}

void PRfabmap::kfCb2(const place_recognizer::keyframeImg::ConstPtr& fmsg)
{
  int camId = 2;
  ROS_INFO("Image Callback Function 2 : Reading Image Number = %d", fmsg->id);

  // Add new keyframe
  place_recognizer::keyframeImg kf_msg;
  kf_msg = *fmsg;
  addNewKeyframeBOW(kf_msg, camId); 
 
}  

bool PRfabmap::isValid() const
{
  return valid;
}



/*
void PRfabmap::computeConfusionMat()
{
  std::vector<cv::of2::IMatch>::iterator l;
  confusionMat = cv::Mat::zeros(bow1.rows, bow2.rows, CV_32F);
  
  for(l= matches.begin(); l!=matches.end();++l)
  {   
      if(l->imgIdx != -1)
	confusionMat.at<float>(l->queryIdx, l->imgIdx) = l->match;   // Probability of matching
      else
	confusionMat.at<float>(l->queryIdx, l->queryIdx) = l->match; // Probability that it is a new image
  }
  
}
*/


