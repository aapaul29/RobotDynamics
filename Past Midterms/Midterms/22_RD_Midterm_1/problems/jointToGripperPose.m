function [ T_IG ] = jointToGripperPose( q, params )
% Inputs:
%     q: a 3x1 vector of generalized coordinates (3, 1).
%     params: a struct of parameters.
%
% Outputs:
%     T_IG: Homogenous transform from inertia to gripper frame (4, 4)

% link lengths (meters)
l01 = params.l01;
l02 = params.l02;
l03 = params.l03;
l04 = params.l04;
l11 = params.l11;
l21 = params.l21;
l22 = params.l22;
% angle (radians)
alpha = params.alpha;

% Joint positions
q0 = q(1);
q1 = q(2);
q2 = q(3);

% Implement your solution here ...
T_IG = zeros(4,4);

end
