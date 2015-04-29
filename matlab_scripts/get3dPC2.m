% Input: dMap = (rows,cols)
% Output: X3d = (rows*cols,3)
function X3d = get3dPC2(dMap)

rows = size(dMap,1);
cols = size(dMap,2);

depth = double(dMap);
% Camera Parameters
f = 525.0;     % Focal length
cX = 319.5;  % Center X
cY = 239.5;  % Center Y
sf = 5000.0;   % Scaling Factor

X3d = zeros(rows*cols,3);
for i = 1:rows
    for j = 1:cols
    
    d = depth(i,j);
    % Compute 3D point
    Z = d/sf;
    X =  ((i-cX)*Z)/f;
    Y =  ((j-cY)*Z)/f;
    P = [X Y Z];
    X3d((i-1)*cols + j,:) = P;
    end
end
    
% Plot 3D Points
%figure;
%showPointCloud(X3d);

end