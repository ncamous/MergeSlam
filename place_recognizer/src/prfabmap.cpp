#include "prfabmap.h"

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
  detector = new cv::StarFeatureDetector(32, 10, 18, 18, 20);
  cv::Ptr<cv::DescriptorExtractor> extractor = new cv::SURF(1000, 4, 2, false, true); // alternative:  cv::SIFT();

  //use a FLANN matcher to generate bag-of-words representations
  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("FlannBased"); // alternative: "BruteForce"
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
  // Convert to cv::Mat using cv_bridge
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
  
  // Compute BOW 
  // 1) Detect Features (parameters in the constructor)
  std::vector<cv::KeyPoint> kpts;
  detector->detect(frame, kpts);
   
  // 2) Compute BOW description of the image
  cv::Ptr<cv::DescriptorExtractor> extractor = new cv::SURF(1000, 4, 2, false, true);// Choose type of descriptor //alternative : SIFT
  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("FlannBased"); //use a FLANN matcher to generate bag-of-words representations // alternative: "BruteForce"
  
  // Temporary implementation for two cameras. (Replace with vector later)
  cv::Mat bow;
  if(camId==1){
    bide->compute(frame, kpts, bow);
    bow1.push_back(bow);
    fId1.push_back(kf_msg.id);
    // Compare with keyframes from camera 2
    if(!bow2.empty())
      compareKeyframeBOW(bow, camId);
    
    }
  else if(camId==2){
    bide->compute(frame, kpts, bow);
    bow2.push_back(bow);
    fId2.push_back(kf_msg.id);
    
    // Compare with keyframes from camera 1
    if(!bow1.empty())
      compareKeyframeBOW(bow,camId);
    }
    
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

