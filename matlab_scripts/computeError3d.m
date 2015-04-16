% Function to compute error in 3D points for given Transformation Matrix
% T12 and T21
function [err12, err21] = computeError3d(X3dC1,X3dC2,T12,T21)

% Convert to homogeneous coordinates
nPts = size(X3dC1,1);
X3dC1 = [X3dC1 , ones(nPts,1)];
X3dC2 = [X3dC2 , ones(nPts,1)];

% Compute err12
 X3dC2_hat = T12*X3dC1';
 err12 = norm(X3dC2_hat - X3dC2');
 err12 = err12/nPts;

% Compute err21
 X3dC1_hat = T21*X3dC2';
 err21 = norm(X3dC1_hat - X3dC1');
 err21 = err21/nPts;
 
end