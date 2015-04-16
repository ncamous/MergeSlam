% Convert from Quaternion to Rotation Matrix
function R = quat2Rot(q)

R11 = q(1)^2 + q(2)^2 - q(3)^2 - q(4)^2; 
R12 = 2*(q(2)*q(3) - q(1)*q(4));
R13 = 2*(q(2)*q(4) + q(1)*q(3));

R21 = 2*(q(3)*q(2) + q(1)*q(4));
R22 = q(1)^2 - q(2)^2 + q(3)^2 - q(4)^2;
R23 = 2*(q(3)*q(4) - q(1)*q(2));

R31 = 2*(q(4)*q(2) - q(1)*q(3));
R32 = 2*(q(4)*q(3) + q(1)*q(2));
R33 = q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2;

R = [R11 R12 R13;
    R21 R22 R23;
    R31 R32 R33];

end