function T01 = jointToTransform01(q)
% Input: joint angles
% Output: homogeneous transformation Matrix from frame 1 to frame 0. T_01
q1 = q(1);
% PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
C_01 = [cos(q1), -sin(q1), 0;
    sin(q1), cos(q1), 0;
    0, 0, 1];
r_01 = [0; 0; 0.145];
T01 = [C_01, r_01;
    zeros(1, 3), 1];
end
