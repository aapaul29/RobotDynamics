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



T_IB = [1 0 0 0;
    0 1 0 q0;
    0 0 1 l02;
    0 0 0 1];
T_BP1 = [1 0 0 0;
    0 cos(q1) -sin(q1) l01;
    0 sin(q1) cos(q1) 0;
    0 0 0 1];
T_P1P2 = [1 0 0 0;
    0 cos(q2) -sin(q2) 0;
    0 sin(q2) cos(q2) l11;
    0 0 0 1];
T_P2G = [1 0 0 0;
    0 cos(-pi/2) -sin(-pi/2) l22;
    0 sin(-pi/2) cos(-pi/2) l21;
    0 0 0 1];
T_IG = T_IB*T_BP1*T_P1P2*T_P2G;

end