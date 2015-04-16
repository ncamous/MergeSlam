% Generate pointcloud from depth image
% Nived Chebrolu
% April 8, 2015
% Note: This program is not efficient for big images (O(N2))

close all; clc; 
% Read the depth image
I = imread('dep1.png');
rows = size(I,1);
cols = size(I,2);
figure;
imshow(I);

% Camera Parameters
f = 525.0;     % Focal length
cX = 319.5;  % Center X
cY = 239.5;  % Center Y
sf = 5000.0;   % Scaling Factor
X3d = [];    % 3D point cloud 

for i = 1:rows
    for j =1:cols
       Z = double(I(i,j))/sf;
       if(Z>0)
            X =  ((i-cX)*Z)/f;
            Y =  ((j-cY)*Z)/f;
            P = [X, Y, Z];
            X3d = [X3d; P]; 
       end    
    end
end

% Plot 3D Points
 figure;
 plot3(X3d(:,1), X3d(:,2), X3d(:,3),'.')



