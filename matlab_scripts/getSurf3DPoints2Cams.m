% Computes 3D Surf Points (Matched) from image(+depth) from two views
function [r_org, r_dis, x2d_org, x2d_dis]  = getSurf3DPoints2Cams(original,dMap_org,distorted,dMap_dis)

% ---------- Compute SURF Features From Images ---------- %
% Load Original + Depth Image
if(ndims(original) == 3)  
    original = rgb2gray(original);
end
figure;
imshow(original);

if(ndims(distorted) == 3)
    distorted = rgb2gray(distorted);
end
figure;
imshow(distorted);


% Find matching between two images
    
% (i) Detect features in both images
ptsOriginal = detectSURFFeatures(original);
ptsDistorted = detectSURFFeatures(distorted);

% (ii) Extract feature descriptors
[featuresOriginal, validPtsOriginal] = extractFeatures(original, ptsOriginal); 
[featuresDistorted, validPtsDistorted] = extractFeatures(distorted, ptsDistorted);

% (iii) Match Features by their Descriptors
indexPairs = matchFeatures(featuresOriginal, featuresDistorted);

% (iv) Retreive locations from images of the corresponding matches
matchedOriginal = validPtsOriginal(indexPairs(:,1));
matchedDistorted = validPtsDistorted(indexPairs(:,2));

figure;
showMatchedFeatures(original,distorted,matchedOriginal,matchedDistorted);
title('Putatively matched points');


% ----------- Find 3D SURF Features --------------------- %
num_matches = matchedOriginal.Count;
% Get depth
r_org = zeros(num_matches,3);
r_dis = zeros(num_matches,3);
for i = 1: num_matches
   r_org(i,:) = get3dPoint(dMap_org, matchedOriginal.Location(i,:));
   r_dis(i,:) = get3dPoint(dMap_dis, matchedDistorted.Location(i,:));
end

x2d_org = matchedOriginal.Location; % Note: (col,row)
x2d_dis = matchedDistorted.Location;% Note: (col,row)

% Remove invalid SURF Points (with zero depth)
% r_org_good = [];
% r_dis_good = [];
% x2d_org_good = [];
% x2d_dis_good = [];
% 
% for i = 1: num_matches
%    
%    if(r_org(i,3) ~= 0 || r_dis(i,3) ~= 0)
%      r_org_good = [r_org_good; r_org(i,:)];     
%      r_dis_good = [r_dis_good; r_dis(i,:)];  
%      x2d_org_good = [x2d_org_good ; matchedOriginal.Location(i,:)]; % Note: (col,row)
%      x2d_dis_good = [ x2d_dis_good ; matchedDistorted.Location(i,:)];% Note: (col,row)
%    end
% 
%  
% end
% 
%  r_org =  r_org_good;
%   r_dis =  r_dis_good;
%   x2d_org = x2d_org_good;
%   x2d_dis = x2d_dis_good;

end