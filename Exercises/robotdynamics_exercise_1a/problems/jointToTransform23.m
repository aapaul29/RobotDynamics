function T23 = jointToTransform23(q)
% Input: joint angles
% Output: homogeneous transformation Matrix from frame 3 to frame 2. T_23

% PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
q3 = q(3);
% Rotation Matrix

C_23 = [cos(q3), 0, sin(q3);
    0, 1, 0;
    -sin(q3), 0, cos(q3)];
% Position Vector
r_23 = [0; 0; 0.27];

T23 = [C_23, r_23;
    zeros(1, 3), 1];
end




