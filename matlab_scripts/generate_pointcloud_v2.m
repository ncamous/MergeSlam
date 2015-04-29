% Generate pointcloud from depth image
% Nived Chebrolu
% April 8, 2015
% Note: This program is not efficient for big images (O(N2))

close all; clc; 
% Read the depth image
I = imread('dep2.png');
C = imread('img2.png');
rows = size(I,1);
cols = size(I,2);
figure;
imshow(I);
depth = double(I);

% Camera Parameters
f = 525.0;     % Focal length
cX = 319.5;  % Center X
cY = 239.5;  % Center Y
sf = 5000.0;   % Scaling Factor
X3d = zeros(rows,cols,3);    % 3D point cloud 


xgrid = ones(rows,1)*(1:cols) - cX;
ygrid = (1:rows)'*ones(1,cols) - cY;
X3d(:,:,1) = xgrid.*depth/f/sf;
X3d(:,:,2) = ygrid.*depth/f/sf;
X3d(:,:,3) = depth/sf;

% Plot 3D Points
figure;
showPointCloud(X3d,C);

% debug info
%distance = sqrt(sum(X3d.^2,1));
%figure;
%hist(distance);
