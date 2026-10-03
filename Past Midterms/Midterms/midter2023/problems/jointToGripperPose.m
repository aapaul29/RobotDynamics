function [ T_IE ] = jointToGripperPose( q, params )
% Inputs:
%     q: a 4x1 vector of generalized coordinates.
%     params: a struct of parameters.
%
% Outputs:
%     T_IE: A 4x4 matrix for homogenous transform from inertial frame {I}
%           to gripper frame {E}

% link lengths (meters)
l0 = params.l0;
l1 = params.l1;
l2 = params.l2;
l3 = params.l3;
l4 = params.l4;
l5 = params.l5;
l6 = params.l6;
l7 = params.l7;

% Joint positions
q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);

% Implement your solution here ...
ry = @(theta) [cos(theta), 0, sin(theta);% RADIANS
               0, 1, 0;% RADIANS
               -sin(theta), 0, cos(theta)];% RADIANS
C_IB = ry(-pi);
t_IB = [l1;0;l0];
T_IB = [C_IB,t_IB;[0 0 0 1]];


C_B0 = eye(3);
t_B0 = [0;0;l2+q0];
T_B0  = [C_B0,t_B0;0 0 0 1];

C_01 = ry(q1);
t_01 = [0;0;l3];
T_01 = [C_01,t_01;0 0 0 1];

C_12 = ry(q2);
t_12 = [0;0;l4];
T_12 = [C_12,t_12;0 0 0 1];


C_23 = ry(q3);
t_23 = [0;0;l5];
T_23 = [C_23,t_23;0 0 0 1];


C_3E = eye(3);
t_3E = [0;0;l6];
T_3E = [C_3E,t_3E;0 0 0 1];

C_EObj = eye(3);
t_EObj = [0;0;l7];
T_EObj = [C_EObj,t_EObj;0 0 0 1];

T_IE = T_IB*T_B0*T_01*T_12*T_23*T_3E;