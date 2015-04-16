#ifndef SIM3SOLVER_H
#define SIM3SOLVER_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <boost/iterator/iterator_concepts.hpp>


class Sim3dSolver{

public:
  
Sim3dSolver(cv::Mat& P1 , cv::Mat& P2);
cv::Mat solve(); // main user interface function
void computeT(cv::Mat& P1, cv::Mat& P2);  

//Set Methods
void setRansacParameters(double prob = 0.99, int minInliers = 6, int maxIterations = 500);

//Get Methods
cv::Mat getEstimatedRot();
cv::Mat getEstimatedTrans();
float getEstimatedScale();

private:

cv::Mat iterateRansac();
void findCentroid(cv::Mat& P, cv::Mat& Pr,  cv::Mat& C);
void computeError3d(cv::Mat& P1, cv::Mat& P2, cv::Mat& T12, cv::Mat& T21, double& err12, double& err21);
void getInliers3d();

// Utils
void quat2Rot(cv::Mat& q, cv::Mat& R);
void randperm(int n, int k, std::vector<int>& idx); 

// Input 3D Points 
cv::Mat X3dC1;
cv::Mat X3dC2;
int numMatches;	// Number of 3D point correspondences

//Current estimation  
cv::Mat R12;
cv::Mat t12;
float s12;
cv::Mat T12;
cv::Mat T21;
std::vector<int> idxInliers;
int numInliers;

// Current best variables
cv::Mat T12_best;
cv::Mat T21_best;
cv::Mat R12_best;
cv::Mat t12_best;
float s12_best;


// RANSAC Parameters
int numData; // Number of random data points to be considered while computing T
double RansacProb;
int RansacMinInlier;
int RansacMaxIts; 
double thresh3d;


};

#endif // SIM3SOLVER_H