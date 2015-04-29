% 3d matrix multiplication 
clear; clc;
close all;

T = rand(3,3);
X = rand(5,5,3);

Y = zeros(5,5,3);

% for i = 1:5
%  for j = 1:5    
%    
%      Y(i,j,:) = X(i,j,:)' 
%  end
% end