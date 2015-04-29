function [idxInliers,numInliers] = getInliers3d(X3dC1,X3dC2,T12,T21,thresh3d)

% Convert to homogeneous coordinates
 nPts = size(X3dC1,1);
 X3dC1 = [X3dC1 , ones(nPts,1)];
 X3dC2 = [X3dC2 , ones(nPts,1)];

% Compute errors for each data point   
%  X3dC2_hat = T12*X3dC1';
%  X3dC1_hat = T21*X3dC2';
%  idxInliers = [];
%  numInliers = 0;
%  
%  for i = 1:nPts
%     err12 = norm(X3dC2_hat(:,i) - X3dC2(i,:)');
%     err21 = norm(X3dC1_hat(:,i) - X3dC1(i,:)');
%      
%       if (err12 < thresh3d && err21< thresh3d)
%          idxInliers = [idxInliers; i];   
%          numInliers = numInliers + 1; 
%      end
% 
%  end
 
 
 % Compute errors for each data point   
 X3dC1_hat = T12*X3dC2';
 X3dC2_hat = T21*X3dC1';
 idxInliers = [];
 numInliers = 0;
 
 for i = 1:nPts
    err12 = norm(X3dC1_hat(:,i) - X3dC1(i,:)');
    err21 = norm(X3dC2_hat(:,i) - X3dC2(i,:)');
     
      if (err12 < thresh3d && err21< thresh3d)
         idxInliers = [idxInliers; i];   
         numInliers = numInliers + 1; 
     end

 end

 
end

