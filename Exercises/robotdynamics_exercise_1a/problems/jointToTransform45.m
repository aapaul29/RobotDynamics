function T45 = jointToTransform45(q)
% Input: joint angles
% Output: homogeneous transformation Matrix from frame 5 to frame 4. T_45

% PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
q5 = q(5);
% Rotation Matrix

C_45 = [cos(q5), 0, sin(q5);
    0, 1, 0;
    -sin(q5), 0, cos(q5)];

% Position Vector
r_45 = [0.168; 0; 0];

T45 = [C_45, r_45;
    zeros(1, 3), 1];
end
