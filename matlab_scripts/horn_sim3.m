% Function to compute the Sim3 transformation using Horn's Algorithm
% Input rl, rr = 3*N

function [T_lr, T_rl, R_lr, t, s] = horn_sim3(rl, rr)

N = length(rl);

%figure;
%plot3(rl(1,:), rl(2,:), rl(3,:),'.')
%hold on;
%plot3(rr(1,:), rr(2,:), rr(3,:),'.r')

% Horn's Algorithm
% ------------- Estimate Rotation Matrix ---------------------- %

% Step 1: Compute Centroid and Relative Coordinates
rl_o = [mean(rl(1,:)) mean(rl(2,:)) mean(rl(3,:))]';
rr_o = [mean(rr(1,:)) mean(rr(2,:)) mean(rr(3,:))]';

rl_n = rl - rl_o*ones(1,N);
rr_n = rr - rr_o*ones(1,N);

% Step 2: Compute M Matrix
%M = rl_n*rr_n';
M = rr_n*rl_n';

% Step 3: Compute N Matrix
N11 = M(1,1) + M(2,2) + M(3,3);
N12 = M(2,3) - M(3,2);
N13 = M(3,1) - M(1,3);
N14 = M(1,2) - M(2,1);
N21 = M(2,3) - M(3,2);
N22 = M(1,1) - M(2,2) - M(3,3);
N23 = M(1,2) + M(2,1);
N24 = M(3,1) + M(1,3);
N31 = M(3,1) - M(1,3);
N32 = M(1,2) + M(2,1);
N33 = -M(1,1)+ M(2,2) - M(3,3);
N34 = M(2,3) + M(3,2);
N41 = M(1,2) - M(2,1);
N42 = M(3,1) + M(1,3);
N43 = M(2,3) + M(3,2);
N44 = -M(1,1) - M(2,2) + M(3,3);

NN = [N11,N12,N13,N14;
    N21,N22,N23,N24;
    N31,N32,N33,N34;
    N41,N42,N43,N44];

% Step 4: Eigenvector of the highest eigenvalue
[V,D] = eig(NN) ;
if ~issorted(diag(D))
    [V,D] = eig(A);
    [D,I] = sort(diag(D));
    V = V(:, I);
end

%Rotation Quaternion
Rq = V(:,4); 

% Quaternion to Rotation Matrix
R_lr = quat2Rot(Rq);

% ------------------ Estimate Scale -------------------------- %
Sl = sum(sum(rl_n.^2));
Sr = sum(sum(rr_n.^2));

% ---- Method 1 ---- %
% D = 0;
% for i = 1:N
%    D = D + rr_n(:,i)'*(R_lr*rl_n(:,i));
% end    
% s_m1 = D/Sl;
% 
% % ---- Method 2 ---- %
% Dbar = 0;
% for i = 1:N
%    Dbar = Dbar + rr_n(:,i)'*(R_lr'*rl_n(:,i));
% end    
% s_m2 = Dbar/Sr; % Scale from right to left 

% ---- Method 1 ---- %
D = 0;
for i = 1:N
   D = D + rl_n(:,i)'*(R_lr*rr_n(:,i));
end    
s_m1 = D/Sr;



% ---- Method 3 ---- %
%s_m3 = sqrt(Sr/Sl);
s_m3 = sqrt(Sl/Sr);


s = s_m3; % Choose one of the three methods

% -------------- Estimate Translation ------------------------ %
%t = rr_o - s*R_lr*rl_o;
t = rl_o - s*R_lr*rr_o;

% -------------- Transformation Matrix ------------------------%
% left to right
T_lr = [s*R_lr, t;
        zeros(1,3),1];

% right to left

sRinv = (1/s)*R_lr';
tinv = -sRinv*t;
T_rl = [sRinv, tinv;
        zeros(1,3),1];

end


