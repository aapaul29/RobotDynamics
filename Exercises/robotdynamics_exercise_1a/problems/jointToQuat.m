function quat = jointToQuat(q)
% Input: joint angles
% Output: quaternion representing the orientation of the end-effector
% q_IE.

% PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
% Rotation Matrix
C_IE = jointToRotMat(q);
% Angle
angle = acos((C_IE(1,1) + C_IE(2,2) + C_IE(3,3) - 1) / 2);
% Normal Vector
n = 1 / (2 * sin(angle)) * [C_IE(3,2) - C_IE(2,3);
    C_IE(1,3) - C_IE(3,1);
    C_IE(2,1) - C_IE(1,2)];
  % Real Part
  xsi_0 = cos(angle / 2);
  % Imaginary Part
  xsi_hat = sin(angle / 2) * n;
  
  quat = [xsi_0;
    xsi_hat];
end