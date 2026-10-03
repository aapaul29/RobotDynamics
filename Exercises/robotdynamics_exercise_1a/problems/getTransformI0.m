function TI0 = getTransformI0()
% Input: void
% Output: homogeneous transformation Matrix from frame 0 to the inertial frame I. T_I0

% Frame 0 coincides with the inertial frame I (I = P0 in Figure 2):
% no rotation (C_I0 = identity) and no offset (I_r_I0 = 0).
C_I0 = eye(3);
I_r_I0 = zeros(3,1);

TI0 = [C_I0,       I_r_I0;
  zeros(1,3), 1     ];
end

