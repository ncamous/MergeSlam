% Test Horn Algorithm with SURF Features 
close all;clc;

% ---------- Compute SURF Features From Images ---------- %
% Load Original + Depth Image
original = imread('img1.png');
dMap_org = imread('dep1.png');
original = rgb2gray(original);
figure;
imshow(original);

distorted = imread('img2.png');
dMap_dis = imread('dep2.png');
distorted = rgb2gray(distorted);
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
% For simple case, assume all of them to be on the same plane
num_matches = matchedOriginal.Count;
%r_org = [matchedOriginal.Location, ones(num_matches,1)];
%r_dis = [matchedDistorted.Location, ones(num_matches,1)];

% Get depth
r_org = zeros(num_matches,3);
r_dis = zeros(num_matches,3);
for i = 1: num_matches
   r_org(i,:) = get3dPoint(dMap_org, matchedOriginal.Location(i,:));
   r_dis(i,:) = get3dPoint(dMap_dis, matchedDistorted.Location(i,:));
  
end

% ---------- Compute Transformation Matrix Using Horn's Algorithm ------- %
[T12, T21, R12, t, s]   = horn_sim3(r_org', r_dis');

% ----------- Verify Computed transformation matrix ----------------------%  

rh_org = [r_org , ones(num_matches,1)];
rh_dis = [r_dis , ones(num_matches,1)];
rh_comp= T12*rh_org';
rh_comp = rh_comp';

figure;
plot3(rh_dis(:,1), rh_dis(:,2), rh_dis(:,3),'.');
hold on;
plot3(rh_comp(:,1), rh_comp(:,2), rh_comp(:,3),'.r');




