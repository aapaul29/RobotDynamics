function T34 = jointToTransform34(q)
% Input: joint angles
% Output: homogeneous transformation Matrix from frame 4 to frame 3. T_34

% PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
q4 = q(4);
% Rotation Matrix

C_34 = [1, 0, 0;
    0, cos(q4), -sin(q4);
    0, sin(q4), cos(q4)];

  % Position Vector
  r_34 = [0.134; 0; 0.07];
  
  T34 = [C_34, r_34;
    zeros(1, 3), 1];
end


