function T12 = jointToTransform12(q)
% Input: joint angles
% Output: homogeneous transformation Matrix from frame 2 to frame 1. T_12

% PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION

q2 = q(2);

% Rotation Matrix
C_12 = [cos(q2), 0, sin(q2);
    0, 1, 0;
    -sin(q2), 0, cos(q2)];
% Position Vector
r_12 = [0; 0; 0.145];

T12 = [C_12, r_12;
    zeros(1, 3), 1];
end