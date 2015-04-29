#include "Sim3dSolver.h"
#include "opencv2/nonfree/nonfree.hpp"
#include <math.h>

Sim3dSolver::Sim3dSolver(cv::Mat& P1, cv::Mat& P2)
{
X3dC1 = P1;
X3dC2 = P2;
numMatches = X3dC1.cols;
setRansacParameters(); // Set Ransac Parameters

// Debug Info: Sim3dSolver Constructor
std::cout<<"X3dC1 size ="<< X3dC1.size() <<std::endl;
std::cout<<"X3dC2 size ="<< X3dC2.size() <<std::endl;
std::cout<<"Num of Matches ="<< numMatches <<std::endl;
}


void Sim3dSolver::setRansacParameters(double prob, int minInliers, int maxIterations)
{
  numData = 3;
  RansacProb = prob;
  RansacMinInlier = minInliers;
  thresh3d = 0.01;
  
  //Change maxIterations based on number of correspondences
  int nIter;
  float eps = (float)minInliers/numMatches;
  nIter =  ceil( log(1-prob) / log(1- std::pow(eps,numData)));
  RansacMaxIts = std::max(1,std::min(maxIterations,nIter));  
   
  // Set error thresholds
}

cv::Mat Sim3dSolver::solve()
{
  if(numMatches < numData)
  {
    std::cout<<"Not enough matches (mininum 3 matches required)"<<std::endl;
    return cv::Mat();
  }    
  else 
    return iterateRansac();  
}



cv::Mat Sim3dSolver::iterateRansac()
{
  
  double err12_best = 1000000.0;
  double err21_best = 1000000.0;
  
  for(int i=0; i < RansacMaxIts; i++)
  {
  
   // Choose 3 (numData) randomly from the set of correspondences 
   std::vector<int> idx;
   randperm(numMatches,numData,idx);
  
   /* // Debug: Random idx genration 
   std::cout<<"Random Indexes Vector = "<<std::endl;
   for(int i=0;i<numData;i++)
     std::cout<<idx[i]<<std::endl;
   */
      
   cv::Mat p3dC1(3,numData,CV_32F);
   cv::Mat p3dC2(3,numData,CV_32F);
   
   for(int i=0;i<numData;i++)
   {
      X3dC1.col(idx[i]).copyTo(p3dC1.col(i));
      X3dC2.col(idx[i]).copyTo(p3dC2.col(i));
   }
   
   // Compute T using random subset
   computeT(p3dC1, p3dC2); 
    
   // Compute Error
   double err12, err21;
   computeError3d(X3dC1,X3dC2,T12,T21,err12,err21);   
   
   
   // Update Best Solution 
   if(err12 < err12_best && err21 < err21_best)
   {
        T12_best = T12;
        T21_best = T21;    
	R12_best = R12;
	t12_best = t12;
	s12_best = s12;
        err12_best = err12;
        err21_best = err21;
	
	// Debug Info:
	std::cout<<"err12_best = "<< err12_best << std::endl;
	std::cout<<"err21_best = "<< err21_best << std::endl;
   }
   
      
  }
  
   //Debug Info: Transformation after Ransac  
   //std::cout<<"T12_best = "<< T12_best << std::endl;
   //std::cout<<"T21_best = "<< T21_best << std::endl;
   //std::cout<<"T Identity Check = "<< T21*T12 << std::endl;
  
   
  return T12_best;
    
}





void Sim3dSolver::computeT(cv::Mat& P1, cv::Mat& P2)
{ // Horn's Algorithm
  // Horn 1987, Closed-form solution of absolute orientataion using unit quaternions  

  // Step 1: Compute Centroid and Relative Coordinates  
  cv::Mat Pr1(P1.rows,P1.cols, P1.type()); // To hold relative coordinates
  cv::Mat Pr2(P2.rows,P2.cols, P2.type());
  cv::Mat O1(3,1,Pr1.type());
  cv::Mat O2(3,1,Pr2.type());
  
  findCentroid(P1,Pr1,O1);
  findCentroid(P2,Pr2,O2);
  
  // Step 2: Compute M Matrix
  cv::Mat M =  Pr1*Pr2.t();
  // Debug M:
  // std::cout<<"Matrix M = "<< M << std::endl;
  
  // Step 3: Compute N Matrix
  double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;
  cv::Mat N(4,4,P1.type());

  N11 = M.at<float>(0,0)+M.at<float>(1,1)+M.at<float>(2,2);
  N12 = M.at<float>(1,2)-M.at<float>(2,1);
  N13 = M.at<float>(2,0)-M.at<float>(0,2);
  N14 = M.at<float>(0,1)-M.at<float>(1,0);
  N22 = M.at<float>(0,0)-M.at<float>(1,1)-M.at<float>(2,2);
  N23 = M.at<float>(0,1)+M.at<float>(1,0);
  N24 = M.at<float>(2,0)+M.at<float>(0,2);
  N33 = -M.at<float>(0,0)+M.at<float>(1,1)-M.at<float>(2,2);
  N34 = M.at<float>(1,2)+M.at<float>(2,1);
  N44 = -M.at<float>(0,0)-M.at<float>(1,1)+M.at<float>(2,2);

  N = (cv::Mat_<float>(4,4) << N11, N12, N13, N14,
                               N12, N22, N23, N24,
                               N13, N23, N33, N34,
                               N14, N24, N34, N44);
  
  // Debug N:
  //std::cout<<"Matrix N = "<< N << std::endl;
  

  // Step 4: Eigenvector of the highest eigenvalue
  cv::Mat eVal, eVec;
  cv::eigen(N,eVal,eVec); // Eigenvalues stored in descending order (eVec[0] is desired quaternion rotation)
  
  /* // Debug eVec:
  std::cout<<"evec = "<< eVec<< std::endl;
  std::cout<<"eval = "<< eVal<< std::endl;
  */
  
   // Convert to rotation matrix
  /*cv::Mat vec(1,3,eVec.type());
  (eVec.row(0).colRange(1,4)).copyTo(vec); //extract imaginary part of the quaternion (sin*axis)
  // Rotation angle. sin is the norm of the imaginary part, cos is the real part
  double ang=atan2(norm(vec),eVec.at<float>(0,0));
  vec = 2*ang*vec/norm(vec); //Angle-axis representation. quaternion angle is the half
  R12.create(3,3,P1.type());
  cv::Rodrigues(vec,R12); // computes the rotation matrix from angle-axis
  // Debug R:
  std::cout<<"(Rodriguez) R12 = "<< R12 << std::endl;
  */
  cv::Mat q(eVec.row(0));
  quat2Rot(q,R12);
  // Debug R12
  //std::cout<<"(quat2Rot) R12 = "<< R12 << std::endl;
  
  // Step 5: Estimate Scale  
  
  cv::Mat aux_Pr1(Pr1.size(),Pr1.type());
  cv::Mat aux_Pr2(Pr2.size(),Pr2.type());
  cv::pow(Pr1,2,aux_Pr1);
  cv::pow(Pr2,2,aux_Pr2);
  
  double S1 =  cv::sum(aux_Pr1)[0];
  double S2 =  cv::sum(aux_Pr2)[0];
  s12 = std::sqrt(S1/S2);
  
  /* // Debug Scale:
  //std::cout<<"Sum aux_Pr1 ="<< S1 << std::endl;
  //std::cout<<"Sum aux_Pr2 ="<< S2 << std::endl; 
  std::cout<<"s12 ="<< s12 << std::endl;
  */
  
  // Step 6: Estimate Translation
  t12.create(3,1,P1.type());
  t12 = O2 - s12*R12*O1;
 
  // Debug Translation
  //std::cout<<"Translation t12 = "<< t12 << std::endl;
  
  // Step 7: Compute Transformation Matrices 
  // T12
  T12 = cv::Mat::eye(4,4,P1.type());
  cv::Mat sR = s12*R12;
  sR.copyTo(T12.rowRange(0,3).colRange(0,3));  
  t12.copyTo(T12.rowRange(0,3).col(3));
  
  //T21
  T21 = cv::Mat::eye(4,4,P1.type());
  cv::Mat sRinv = (1.0/s12)*R12.t();
  sRinv.copyTo(T21.rowRange(0,3).colRange(0,3));
  cv::Mat tinv = -sRinv*t12;
  tinv.copyTo(T21.rowRange(0,3).col(3));
  
  /* // Transformation Matrices
  std::cout<<"T12 = "<< T12 << std::endl;
  std::cout<<"T21 = "<< T21 << std::endl;
  std::cout<<"Identity Check = "<< T21*T12 << std::endl;
  */
  
}


void Sim3dSolver::findCentroid(cv::Mat& P, cv::Mat& Pr, cv::Mat& C)
{
    cv::reduce(P,C,1,CV_REDUCE_SUM);
    C = C/P.cols;

    for(int i=0; i<P.cols; i++)
    { 
      Pr.col(i)=P.col(i)-C;
    }
 
  /* // Debug Code:
  std::cout<<"Centroid = "<<C<<std::endl;
  std::cout<<"Rel Coordinates : Rows ="<<Pr.rows <<", Cols ="<<Pr.cols << std::endl;
  std::cout<<" Rel Coordinates sum check: "<< cv::sum(Pr) <<std::endl;
  */
}

void Sim3dSolver::computeError3d(cv::Mat& P1, cv::Mat& P2, cv::Mat& T12, cv::Mat& T21, double& err12, double& err21)
{
 
int nPts = P1.cols;

// Convert to homogeneous coordinates  
cv::Mat Ph1 = cv::Mat::ones(4, nPts, P1.type());
P1.copyTo(Ph1.rowRange(0,3).colRange(0,nPts));
cv::Mat Ph2 = cv::Mat::ones(4,nPts, P2.type());
P2.copyTo(Ph2.rowRange(0,3).colRange(0,nPts));


// Compute err12
cv::Mat P2_hat(4,nPts, P2.type());
P2_hat = T12*Ph1; 
err12 = cv::norm(P2_hat,Ph2);
err12 = err12/nPts;

// Compute err21
cv::Mat P1_hat(4,nPts, P1.type());
P1_hat = T21*Ph2; 
err21 = cv::norm(P1_hat,Ph1);
err21 = err21/nPts;
 
}


void Sim3dSolver::getInliers3d( )
{
  // Convert to homogeneous coordinates  
  cv::Mat Ph1 = cv::Mat::ones(4, numMatches, X3dC1.type());
  X3dC1.copyTo(Ph1.rowRange(0,3).colRange(0,numMatches));
  cv::Mat Ph2 = cv::Mat::ones(4,numMatches, X3dC2.type());
  X3dC2.copyTo(Ph2.rowRange(0,3).colRange(0,numMatches));
 
  // Compute errors for each data point
  cv::Mat P2_hat(4,numMatches, X3dC2.type());
  P2_hat = T12*Ph1; 
 
  cv::Mat P1_hat(4,numMatches, X3dC1.type());
  P1_hat = T21*Ph2;

  double e12 = 0.0;
  double e21 = 0.0;
  idxInliers.clear();
  
  for(int i=0;i<numMatches;i++)
  {   
      e12 = cv::norm(P2_hat.col(i) - X3dC2.col(i));
      e21 = cv::norm(P1_hat.col(i) - X3dC1.col(i));
      
      if(e12 < thresh3d && e21 < thresh3d)
      {
	 idxInliers.push_back(i);
	 numInliers = numInliers + 1;
      }
  }
  
}


// Utils

void Sim3dSolver::quat2Rot(cv::Mat& q, cv::Mat& R)
{
  R.create(3,3,CV_32F);
  double R11, R12, R13, R21, R22, R23, R31, R32, R33;
  
  R11 = q.at<float>(0,0)*q.at<float>(0,0) + q.at<float>(0,1)*q.at<float>(0,1) - q.at<float>(0,2)*q.at<float>(0,2) - q.at<float>(0,3)*q.at<float>(0,3);
  R12 = 2*(q.at<float>(0,1)*q.at<float>(0,2) - q.at<float>(0,0)*q.at<float>(0,3));
  R13 = 2*(q.at<float>(0,1)*q.at<float>(0,3) + q.at<float>(0,0)*q.at<float>(0,2));
  
  R21 = 2*(q.at<float>(0,2)*q.at<float>(0,1) + q.at<float>(0,0)*q.at<float>(0,3));
  R22 = q.at<float>(0,0)*q.at<float>(0,0) - q.at<float>(0,1)*q.at<float>(0,1) + q.at<float>(0,2)*q.at<float>(0,2) - q.at<float>(0,3)*q.at<float>(0,3);
  R23 = 2*(q.at<float>(0,2)*q.at<float>(0,3) - q.at<float>(0,0)*q.at<float>(0,1));
  
  R31 = 2*(q.at<float>(0,3)*q.at<float>(0,1) - q.at<float>(0,0)*q.at<float>(0,2));
  R32 = 2*(q.at<float>(0,3)*q.at<float>(0,2) + q.at<float>(0,0)*q.at<float>(0,1));
  R33 = q.at<float>(0,0)*q.at<float>(0,0) - q.at<float>(0,1)*q.at<float>(0,1) - q.at<float>(0,2)*q.at<float>(0,2) + q.at<float>(0,3)*q.at<float>(0,3);

  R = (cv::Mat_<float>(3,3) << R11, R12, R13,
                               R21, R22, R23,
                               R31, R32, R33);
  
}

void Sim3dSolver::randperm(int n, int k, std::vector<int>& idx)
{
  std::vector<int> in;
  for(int i=0; i<n; i++)
      in.push_back(i);
  
  cv::randShuffle(in);
  
  for(int i=0;i<k;i++)
      idx.push_back(in[i]);      
}

// Get Methods
cv::Mat Sim3dSolver::getEstimatedRot()
{
 return R12_best.clone();  
}

cv::Mat Sim3dSolver::getEstimatedTrans()
{
  return t12_best.clone();
}

float Sim3dSolver::getEstimatedScale()
{
  return s12_best;
}

