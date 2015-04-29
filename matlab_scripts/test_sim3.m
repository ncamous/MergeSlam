%% Test Sim3 Estimation
clc;
close all;

% Each column is a point
x1 = [-1,1,0;  
       0,0,1; 
       0,0,0];


% Test 
theta = deg2rad(30);
s = 2;
t = [4,0,0]';
R = [ cos(theta), -sin(theta), 0;
      sin(theta), cos(theta),  0;
      0,          0,          1];
  
T_test = [ s*R, t;
           zeros(1,3), 1 ];
x1h = [x1 ;
       ones(1,3)];

x2h = T_rot*x1h;
x2    = x2h(1:3,:);   

% Compute Transformation
[T12, T21, R12, t12, s12] = horn_sim3(x1,x2);








