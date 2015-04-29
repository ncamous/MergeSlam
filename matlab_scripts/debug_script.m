% Script Fo Debugging Purposes
close all; clc;

% Load Original + Depth Image
original = imread('img1.png');
dMap_org = imread('dep1.png');
original = rgb2gray(original);
figure;
imshow(original);

% Compute a 3D Point For a known pixel location  
x_loc = [25,60]
P = get3dPoint(dMap_org,x_loc);

% Project 3D Point onto image and check if its the same location
xp_loc = projectIntoImage(P)
