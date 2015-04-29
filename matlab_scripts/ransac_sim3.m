% Compute Sim3 Transformation with RANSAC
% Nived Chebrolu
% April 9, 2015
close all; clc;

% Load Images
im1 = imread('img1.png');
dMap1 = imread('dep1.png');

im2 = imread('img1_rot30.png');
dMap2 = imread('dep1_rot30.png');

% RANSAC parameters
numData = 3;
minInliers = 6; 
maxIter = 1000; 
dThresh = 0.9; %ratio of inliers to total data points
thresh3d = 0.01; %in cm

% Current Ransac State
err12_best = 10000000;
err21_best = 10000000;

% Load 3D correspondences from two cameras
[X3dC1, X3dC2, X2dC1, X2dC2]  = getSurf3DPoints2Cams(im1,dMap1,im2,dMap2); %3D SURF points in Cam1 and Cam2 
numPts = length(X3dC1);

% Compute number of iterations based on number of correspondences
prob = 0.99; %Probability that algorithm gives an acceptable answer
eps = minInliers/numPts;
nIter = ceil(log(1-prob)/log(1-eps^numData));
N = min(nIter, maxIter); % Choose the lower between maxIter and nIter (given by prob bound) 

% Initializations
T12_best = [eye(3,3), zeros(3,1); 
            zeros(1,3), 1 ];
T21_best = [eye(3,3), zeros(3,1); 
            zeros(1,3), 1 ];
R12_best = eye(3,3);
t12_best = zeros(3,1);
s12_best = 1;
idxInliers_best = [];
numInliers_best = 0;
   
% Iterate over random samples (min size = 3) 
for i = 1:N
    
   % Choose 3 data points randomly from each set 
    idx = randperm(numPts,numData);
    r1 =  X3dC1(idx,:); 
    r2 =  X3dC2(idx,:);
   
   % Compute T
   [T12, T21, R12, t12, s12] = horn_sim3(r1',r2');   
   
%    % Debugging -- Check whether transformation computed is reasonable
     % T12
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
   if(err12 < err12_best && err21 < err21_best && numInliers>=minInliers)
        T12_best = T12;
        T21_best = T21;
        R12_best = R12;
        t12_best = t12;
        s12_best = s12;
        idxInliers_best = idxInliers;
        numInliers_best = numInliers;
        err12_best = err12;
        err21_best = err21;
   end
  
  if(numInliers/numPts > dThresh)
      [T12_best, T21_best, ~, ~, ~] = horn_sim3(X3dC1(idxInliers,:)',X3dC2(idxInliers,:)');
      [err1_dThresh, err2_dThresh] = computeError3d(X3dC1,X3dC2,T12_best,T21_best);
     break;   
  end 
  
end

% Refine update using all inlier points
if(~isempty(idxInliers_best))
    [T12_best, T21_best, ~, ~, ~] = horn_sim3(X3dC1(idxInliers_best,:)',X3dC2(idxInliers_best,:)');
end

% theta = deg2rad(30);
% T30 = [cos(theta) -sin(theta) 0 0;
%        sin(theta) cos(theta) 0  0;
%        0            0        1  0;
%        0            0         0 1];
% 
% 
% % Debugging -- Check whether transformation computed is reasonable
%     % T12
%     rh1 = [X3dC1 , ones(numPts,1)];
%     rh2 = [X3dC2 , ones(numPts,1)];
%     rh2_hat= rh1*T30;
%     rh2_hat = rh2_hat';
%     
%     figure;
%     plot3(rh1(:,1), rh1(:,2), rh1(:,3),'.r');
%     hold on;
%     plot3(rh2(:,1), rh2(:,2), rh2(:,3),'.');
%         
%     figure;
%     plot3(rh2(:,1), rh2(:,2), rh2(:,3),'.');
%     hold on;
%     plot3(rh2_hat(:,1), rh2_hat(:,2), rh2_hat(:,3),'.r');
%     hold on;
%     title('r2(blue) vs r2hat(red)');
%    
% %    % T21
% %    rh1_hat= T21*rh2';
% %    rh1_hat = rh1_hat';
% %    figure;
% %    plot3(rh1(:,1), rh1(:,2), rh1(:,3),'.');
% %    hold on;
% %    plot3(rh1_hat(:,1), rh1_hat(:,2), rh1_hat(:,3),'.r');
% %    hold on;
% %    title('r1hat vs r1');
% 
% % Merge two pointclouds using Transformation
% % Load Original + Depth Image
% % Compute Pointcloud 1 from depth image 1
% X1_PC = get3dPC(dMap1);
% 
% % Compute Pointcloud 2 from depth image 2
% X2_PC = get3dPC(dMap2);
%     
 
% % Plot both pointclouds together
% figure;
% showPointCloud(X1_PC,[1,0,0]);
% hold on;
% showPointCloud(X2_PC,[0,0,1]);
% 
% figure;
% showPointCloud(X2_PC_hat,[1,0,0]);
% hold on;
% showPointCloud(X2_PC,[0,0,1]);

% ----------------------------------------------------------------------------%

theta = deg2rad(-30);
T30 = [cos(theta) -sin(theta) 0 0;
       sin(theta) cos(theta) 0  0;
       0            0        1  0;
       0            0         0 1];

T30inv = [cos(-theta) -sin(-theta) 0 0;
       sin(-theta) cos(-theta) 0  0;
       0            0        1  0;
       0            0         0 1];

   
% Debugging -- Check whether transformation computed is reasonable
    % T12
    rh1 = [X3dC1 , ones(numPts,1)];
    rh2 = [X3dC2 , ones(numPts,1)];
    rh1_hat = T12_best*rh2';
    rh1_hat = rh1_hat';
   
    figure;
    plot3(rh1(:,1), rh1(:,2), rh1(:,3),'.r');
    hold on;
    plot3(rh2(:,1), rh2(:,2), rh2(:,3),'.');
    hold on;
    title('r1(red) vs r2(blue)');    
    
    
    figure;
    plot3(rh1(:,1), rh1(:,2), rh1(:,3),'.r');
    hold on;
    plot3(rh1_hat(:,1), rh1_hat(:,2), rh1_hat(:,3),'.');
    hold on;
    title('r1(red) vs r1hat(blue)');
    
    
    % T12
%     rh1 = [X3dC1 , ones(numPts,1)];
%     rh2 = [X3dC2 , ones(numPts,1)];
%     rh1_hat = rh2*T12_best;
%        
%     figure;
%     plot3(rh1(:,1), rh1(:,2), rh1(:,3),'.r');
%     hold on;
%     plot3(rh2(:,1), rh2(:,2), rh2(:,3),'.');
%     hold on;
%     title('r1(red) vs r2(blue)');    
%     
%     
%     figure;
%     plot3(rh1(:,1), rh1(:,2), rh1(:,3),'.r');
%     hold on;
%     plot3(rh1_hat(:,1), rh1_hat(:,2), rh1_hat(:,3),'.');
%     hold on;
%     title('r1(red) vs r1hat(blue)');
    
 
% Merge two pointclouds using Transformation
% Load Original + Depth Image
% Compute Pointcloud 1 from depth image 1
X1_PC = get3dPC2(dMap1);
nPC1 = length(X1_PC);
X1h_PC = [X1_PC ones(nPC1,1)];

% Compute Pointcloud 2 from depth image 2
X2_PC = get3dPC2(dMap2);
nPC2 = length(X2_PC);
X2h_PC = [X2_PC ones(nPC2,1)];

% Transform Pointlcloud 1 using T12
% X1_PC_hat = transformPC(X2_PC,T12_best);
X1h_PC_hat = T12_best*X2h_PC';
X1_PC_hat = X1h_PC_hat(1:3,:)';

%Plot both pointclouds together
% figure;
% showPointCloud(X1_PC,[1,0,0]);
% hold on;
% showPointCloud(X2_PC,[0,0,1]);
% hold on;
% title('r1(red) vs r2(blue)'); 
% 
% figure;
% showPointCloud(X1_PC,[1,0,0]);
% hold on;
% showPointCloud(X1_PC_hat,[0,0,1]);
% hold on;
% title('r1(red) vs r1hat(blue)');

% figure;
% showPointCloud(X1_PC,im1);
% hold on;
% showPointCloud(X2_PC,im2);
% hold on;
% title('r1(red) vs r2(blue)'); 
% 
% figure;
% showPointCloud(X1_PC,im1);
% hold on;
% showPointCloud(X1_PC_hat,im2);
% hold on;
% title('X1_PC vs X1_PC_hat');


% Compute Transformation Again (using complete pointcloud)
% r1n = getPCPoints(X1_PC);
% r1n_hat = getPCPoints(X2_PC);
% 
% [T12n , T21n, R12n, t12n, s12n] = horn_sim3(r1n', r1n_hat');
% 
% % New transformed point cloud
% X1n_PC_hat = transformPC(X2_PC,T12n);
% 
% figure;
% showPointCloud(X1_PC,im1);
% hold on;
% showPointCloud(X1n_PC_hat,im2);
% hold on;
% title('X1_PC vs X1n_PC_hat)');



figure;
myShowPointCloud(X1_PC,im1);
hold on;
myShowPointCloud(X2_PC,im2);
hold on;
title('X1 vs X2')

figure;
myShowPointCloud(X1_PC,im1);
hold on;
myShowPointCloud(X1_PC_hat,im2);
hold on;
title('X1 vs X1-hat');



