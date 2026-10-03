function [ J_IE_p ] = jointToPositionJacobian(q, params)
% Inputs:
%     q: a 4x1 vector of generalized coordinates
%     params: a struct of parameters
%
% Outputs:
%     J_IE_p: 3x4 position Jacobian of gripper frame {E} wrt inertia frame {I}
%             in inertial frame {I}

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

% Implement your solution here...

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

% new
T_I0 = T_IB*T_B0;
T_I1 = T_I0*T_01;
T_I2 = T_I1*T_12;
T_I3 = T_I2*T_23;
T_IE = T_I3*T_3E;

% Extract the rotation matrices from each homogeneous transformation
% matrix.
R_I0= T_I0(1:3,1:3);
R_I1 = T_I1(1:3,1:3);
R_I2 = T_I2(1:3,1:3);
R_I3 = T_I3(1:3,1:3);
R_IE = T_IE(1:3,1:3);

% Extract the position vectors from each homogeneous transformation
% matrix.
r_I_I0 = T_I0(1:3,4);
r_I_I1 = T_I1(1:3,4);
r_I_I2 = T_I2(1:3,4);
r_I_I3 = T_I3(1:3,4);


% Define the unit vectors around which each link rotates in the precedent
% coordinate frame.
n_1 = [0 0 0]';
n_2 = [0 1 0]';
n_3 = [0 1 0]';
n_4 = [0 1 0]';

r_I_IE = T_IE(1:3,4);

% Compute the translational jacobian.
J_IE_p = [cross(R_I0*n_1, r_I_IE - r_I_I0) ...
    cross(R_I1*n_2, r_I_IE - r_I_I1)...
    cross(R_I2*n_3, r_I_IE - r_I_I2) ...
    cross(R_I3*n_4, r_I_IE - r_I_I3) ...
    ];
J_IE_p = J_IE_p +[[0;0;-1],zeros(3,3)];