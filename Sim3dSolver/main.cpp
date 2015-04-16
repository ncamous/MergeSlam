#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/nonfree/nonfree.hpp"
#include "Sim3dSolver.h"

void compute3dPoint(cv::Mat& dep, cv::Mat& p2d, cv::Mat& p3d);


int main(){
  
  // Load Images + Corresponding depth Maps
  cv::Mat img1 = cv::imread("img1.png",CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat dep1 = cv::imread("dep1.png",CV_LOAD_IMAGE_UNCHANGED);
  cv::Mat img2 = cv::imread("img1_rot30.png",CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat dep2 = cv::imread("dep1_rot30.png",CV_LOAD_IMAGE_UNCHANGED);
  
  //std::cout << "Depth Type= " << dep1.type() << std::endl;
  /*// Debug Info: Depth
  double minVal, maxVal;
  cv::minMaxLoc( dep1, &minVal, &maxVal );
  std::cout << "minVal = " << minVal << std::endl;
  std::cout << "maxVal = " << maxVal << std::endl;
  */
  
  /*
  // Visualize Images and depth map
  cv::namedWindow("Image1");
  cv::imshow("Image1",img1);
  cv::waitKey(1);
  cv::namedWindow("Image2");
  cv::imshow("Image2",img2);
  cv::waitKey(1);
  cv::namedWindow("Depth1");
  cv::imshow("Depth1",dep1);
  cv::waitKey(1);
  cv::namedWindow("Depth2");
  cv::imshow("Depth2",dep2);
  cv::waitKey(1);
  */
    
  // Detect features in the Image1
  // -- Step 1: Detect the keypoints using SURF Detector
  // Detector Type 1
  int minHessian = 400;
  cv::SurfFeatureDetector detector( minHessian );
 
  // Detector type 2
  //cv::StarFeatureDetector detector(32, 10, 18, 18, 20);
  
  std::vector<cv::KeyPoint> keypoints_img1, keypoints_img2;
  detector.detect( img1, keypoints_img1 );
  detector.detect( img2, keypoints_img2 );
  std::cout << "Number of features in Image 1:" <<keypoints_img1.size()<<std::endl;
  std::cout << "Number of features in Image 1:" <<keypoints_img2.size()<<std::endl;
  
  
  /* // For Debugging : Visualize keypoints
  static const std::string FEATURES_DEBUG_WINDOW1 = "Cam 1";
  static const std::string FEATURES_DEBUG_WINDOW2 = "Cam 2";
  cv::drawKeypoints(img1, keypoints_img1, img1);
  cv::imshow(FEATURES_DEBUG_WINDOW1, img1);
  cv::waitKey(1);
  cv::drawKeypoints(img2, keypoints_img2, img2);
  cv::imshow(FEATURES_DEBUG_WINDOW2, img2);
  cv::waitKey(0); 
    */
  
  // -- Step 2: Compute Descriptors (Feature Vectors)
  cv::SurfDescriptorExtractor extractor;
  cv::Mat descriptors_img1, descriptors_img2;
  extractor.compute( img1, keypoints_img1, descriptors_img1 );
  extractor.compute( img2, keypoints_img2, descriptors_img2 );
  
  // -- Step 3: Compute matches and corresponding feature locations
  cv::FlannBasedMatcher matcher;
  std::vector< cv::DMatch > matches;
  matcher.match( descriptors_img1, descriptors_img2, matches);
 
  // Compute good matches (Need to change this)
  double max_dist = 0; double min_dist = 100;
  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_img1.rows; i++ )
  { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }

  std::cout<<"-- Max dist :"<<max_dist<< std::endl;
  std::cout<<"-- Min dist :"<<min_dist<< std::endl;

  //-- Consider only "good" matches (i.e. whose distance is less than 3*min_dist )
  std::vector< cv::DMatch > good_matches;
  for( int i = 0; i < descriptors_img1.rows; i++ )
  { if( matches[i].distance <= 5*min_dist )
    {  good_matches.push_back( matches[i]); }
  }

  /* Debug: Good matches Img locations
  for(int i=1; i<5; i++)
    {
      std::cout<<"Query Index="<< good_matches[i].queryIdx <<", Train Index = "<< good_matches[i].trainIdx<< std::endl;
      int idx1 = good_matches[i].queryIdx;
      int idx2 = good_matches[i].trainIdx;
      std::cout<<"Loc1="<<keypoints_img1[idx1].pt <<", Loc2 = "<< keypoints_img2[idx2].pt<< std::endl;
    }  
  */
  
  // Extract corresponding keypoints from two images and store their image locations
  std::vector<cv::DMatch>::iterator l;
  std::vector<cv::Point2f> p2dC1;
  std::vector<cv::Point2f> p2dC2;
  
  for(l= good_matches.begin(); l!= good_matches.end(); l++)
  { int idx1 = l->queryIdx;
    int idx2 = l->trainIdx;
    p2dC1.push_back(keypoints_img1[idx1].pt);
    p2dC2.push_back(keypoints_img2[idx2].pt);
  }
  cv::Mat X2dC1(p2dC1);
  cv::Mat X2dC2(p2dC2);
  
  // Compute 3D points corresponding to 2D image locations and depth map
  cv::Mat X3dC1;
  cv::Mat X3dC2;
  compute3dPoint(dep1,X2dC1,X3dC1);
  compute3dPoint(dep2,X2dC2,X3dC2);
  
   /* // Debug 2d/3d points
   std::cout<<"Debug Info: X3dC1 Size"<< X3dC1.rows << "," << X3dC1.cols << ", Type"<< X3dC1.type() << std::endl;
   std::cout<<"Debug Info: X3dC2 Size"<< X3dC2.rows << "," << X3dC2.cols << ", Type"<< X3dC2.type() << std::endl;
   for (int i = 0; i < X3dC1.rows; i++)
   { std::cout<<" Cam1= "<<X3dC1.at<float>(i,0)<<","<<X3dC1.at<float>(i,1)<<","<<X3dC1.at<float>(i,2)<<" Cam2= "<<X3dC2.at<float>(i,0)<<","<<X3dC2.at<float>(i,1)<<","<<X3dC2.at<float>(i,2)<<std::endl;
   }
  */
   
  // Compute Similarity Transformation between X3dC1 and X3dC2
  cv::Mat X3dC1t(X3dC1.t());
  cv::Mat X3dC2t(X3dC2.t());
  //std::cout<<"Debug Info: X3dC1t Rows = "<< X3dC1t.rows << ", Cols =" << X3dC1t.cols << ", Type"<< X3dC1t.type() << std::endl;
  
  // Compute Transformation 
  Sim3dSolver sim(X3dC1t,X3dC2t);
  cv::Mat T12 = sim.solve();
  //sim.computeT(X3dC1t,X3dC2t);
  
  std::cout<<"Estimated Transformation Matrix :"<<std::endl<<T12<<std::endl;
  std::cout<<"Estimated Rotation Matrix :"<<std::endl<<sim.getEstimatedRot()<<std::endl;
  std::cout<<"Estimated Translation :"<<std::endl<<sim.getEstimatedTrans()<<std::endl;
  std::cout<<"Estimated scale :"<<sim.getEstimatedScale()<<std::endl;
  
  // Visualize feature matches between two images
  cv::Mat img_matches;
  cv::drawMatches( img1, keypoints_img1, img2, keypoints_img2,
               good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
               std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

  cv::imshow( "Good Matches", img_matches );
  cv::waitKey(0);
   
  return 0;  
}



void compute3dPoint(cv::Mat& dMap, cv::Mat& p2d, cv::Mat& p3d)
{
  p3d.create(p2d.rows,3,CV_32F);
  
  // Camera Parameters
  float f = 525.0;   // Focal length
  float cX = 319.5;  // Center X
  float cY = 239.5;  // Center Y
  float sf = 5000.0; // Scaling Factor
  
  //Interest Point Location
  int u = 0;
  int v = 0;
  float d = 0.0;
  float X,Y,Z;
   
  for(int i = 0; i< p2d.rows; i++)
  { 
    u =  static_cast<int>(p2d.at<float>(i,1));
    v =  static_cast<int>(p2d.at<float>(i,0));
    d =  static_cast<float>(dMap.at<unsigned short>(u,v)); 
			  
    Z = d/sf;
    X =  ((u-cX)*Z)/f;
    Y =  ((v-cY)*Z)/f;
    
   p3d.at<float>(i,0) = X;
   p3d.at<float>(i,1) = Y;
   p3d.at<float>(i,2) = Z;
   
    
  }
  
}  
