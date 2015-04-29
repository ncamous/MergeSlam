% Rotate Image
clc; clear all;
close all;

% Load Original Image
original = imread('dep1.png');
imshow(original);
%imwrite(original,'cameraman.jpg')

%Distort the image
scale = 1;
%J = imresize(original, scale);
theta = 45;
distorted = imrotate(original,theta,'crop');
figure, imshow(distorted)
%distorted = rgb2gray(distorted);
imwrite(distorted,'dep1_rot45.png')
