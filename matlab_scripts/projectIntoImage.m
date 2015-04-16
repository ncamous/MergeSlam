% Project 3D Points into 2D points in the image using Camera Instrinsic
% Parameters
% Input: X3d = numPts*3
% Output: X2d = numPts*2
% Nived Chebrolu
% April 9, 2015

function X2d = projectIntoImage(X3d)
 numPts = size(X3d,1); % Total number of points

 % Camera Parameters (Pinhole model)
 f  = 525.0;     % Focal length
 cX = 319.5;     % Center X
 cY = 239.5;     % Center Y
 for i = 1:numPts
    x = X3d(i,1);
    y = X3d(i,2);
    z = X3d(i,3);
    
    if(z~=0)
       u = (x*f)/z + cX;     
       v = (y*f)/z + cY;
       X2d(i,1) = v;
       X2d(i,2) = u;
    else
       X2d(i,:) = [0,0];
    end 
 end  

end