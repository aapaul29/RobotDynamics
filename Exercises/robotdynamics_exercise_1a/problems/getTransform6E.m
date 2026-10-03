function T6E = getTransform6E()
% Input: void
% Output: homogeneous transformation Matrix from the end-effector frame E to frame 6. T_6E

% PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
% Rotation Matrix
C_6E = eye(3);
% Position Vector
r_6E = zeros(3, 1);

T6E = [C_6E, r_6E;
    zeros(1, 3), 1];
end

