function [ J_IG_p ] = jointToPositionJacobian(q, params)
% Inputs:
%     q: a 3x1 vector of generalized coordinates
%     params: a struct of parameters
%
% Outputs:
%     J_IG_p: position Jacobian of gripper in inertia frame (3,3)

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

% Implement your solution here...
J_IG_p = zeros(3,3);

end
