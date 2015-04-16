% Compute Sim3 Transformation with RANSAC
% Nived Chebrolu
% April 9, 2015
close all; clc;

% RANSAC parameters
numData = 3;
minInliers = 6; 
maxIter = 1000; 
dThresh = 0.8; %ratio of inliers to total data points
thresh3d = 0.01; %in cm

% Current Ransac State
T12_best = [];
T21_best = [];
err12_best = 10000000;
err21_best = 10000000;

% Load 3D correspondences from two cameras
[X3dC1, X3dC2, X2dC1, X2dC2]  = getSurf3DPoints2Cams(); %3D SURF points in Cam1 and Cam2 
numPts = length(X3dC1);

% Compute number of iterations based on number of correspondences
prob = 0.99; %Probability that algorithm gives an acceptable answer
eps = minInliers/numPts;
eps = minInliers/30;
nIter = ceil(log(1-prob)/log(1-eps^numData));
N = min(nIter, maxIter); % Choose the lower between maxIter and nIter (given by prob bound) 

% Iterate over random samples (min size = 3) 
for i = 1:N
    
   % Choose 3 data points randomly from each set 
    idx = randperm(numPts,numData);
    r1 =  X3dC1(idx,:); 
    r2 =  X3dC2(idx,:);
   
   % Compute T
   [T12, T21, R12, t12, s12] = horn_sim3(r1',r2');   
  
%    % Debugging -- Check whether transformation computed is reasonable
%    % T12
%    rh1 = [r1 , ones(numData,1)];
%    rh2 = [r2 , ones(numData,1)];
%    rh2_hat= T12*rh1';
%    rh2_hat = rh2_hat';
%    figure;
%    plot3(rh2(:,1), rh2(:,2), rh2(:,3),'.');
%    hold on;
%    plot3(rh2_hat(:,1), rh2_hat(:,2), rh2_hat(:,3),'.r');
%    hold on;
%    title('r2hat vs r2');
%    
%    % T21
%    rh1_hat= T21*rh2';
%    rh1_hat = rh1_hat';
%    figure;
%    plot3(rh1(:,1), rh1(:,2), rh1(:,3),'.');
%    hold on;
%    plot3(rh1_hat(:,1), rh1_hat(:,2), rh1_hat(:,3),'.r');
%    hold on;
%    title('r1hat vs r1');
    
  
   % Compute error - Use any error criteria
   % Error in 3D SURF Points
   [err12, err21] = computeError3d(X3dC1,X3dC2,T12,T21);
   [idxInliers,numInliers] = getInliers3d(X3dC1,X3dC2,T12,T21,thresh3d);
   
   % Error in 2D SURF Points
   %[err12, err21] = computeError2d(X3dC1,X3dC2,X2dC1,X2dC2,T12,T21);
   
   % Update best solution
   if(err12 < err12_best && err21 < err21_best)
        T12_best = T12;
        T21_best = T21;    
        err12_best = err12
        err21_best = err21
   end
  
  if(numInliers/numPts > dThresh)
      [T12_best, T21_best, ~, ~, ~] = horn_sim3(X3dC1(idxInliers,:)',X3dC2(idxInliers,:)');
      [err1_dThresh, err2_dThresh] = computeError3d(X3dC1,X3dC2,T12_best,T21_best)
     break;   
  end 
  
end