function T56 = jointToTransform56(q)
% Input: joint angles
% Output: homogeneous transformation Matrix from frame 6 to frame 5. T_56

% PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
q6 = q(6);
% Rotation Matrix
C_56 = [1, 0, 0;
  0, cos(q6), -sin(q6);
  0, sin(q6), cos(q6)];

% Position Vector
r_56 = [0.072; 0; 0];

T56 = [C_56, r_56;
  zeros(1, 3), 1];
end


