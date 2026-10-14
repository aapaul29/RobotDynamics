function TI0 = getTransformI0()
% Input: void
% Output: homogeneous transformation Matrix from frame 0 to the inertial frame I. T_I0

% PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
C_I0 = eye(3);
r_I0 = zeros(3, 1);
TI0 = [C_I0, r_I0;
    zeros(1, 3), 1];
end
