function X3d = get3dPC(dMap)

rows = size(dMap,1);
cols = size(dMap,2);

depth = double(dMap);
% Camera Parameters
f = 525.0;     % Focal length
cX = 319.5;  % Center X
cY = 239.5;  % Center Y
sf = 5000.0;   % Scaling Factor

% 3D point cloud
X3d = zeros(rows,cols,3);     
xgrid = ones(rows,1)*(1:cols) - cX;
ygrid = (1:rows)'*ones(1,cols) - cY;
X3d(:,:,1) = xgrid.*depth/f/sf;
X3d(:,:,2) = ygrid.*depth/f/sf;
X3d(:,:,3) = depth/sf;

% Plot 3D Points
%figure;
%showPointCloud(X3d);

end