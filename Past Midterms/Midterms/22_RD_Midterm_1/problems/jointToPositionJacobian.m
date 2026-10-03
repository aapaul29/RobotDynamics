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

% Note: Formula in the script, eq. 2.155, is only applicable
%     for rotating joints, which are on the arm. For the base that
%     only translates, we can account for its component directly.
%
% We apply eq. 2.154: J_IG_I = J_IB_I + J_BG_I
%
% Since, B is not a rotating frame, J_BG_B = J_BG_I
%%

% Compute the relative homogeneous transformation matrices.
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
% Compute the homogeneous transformation matrices from frame k to the
% inertial frame I.
T_IP1 = T_IB*T_BP1;
T_IP2 = T_IP1*T_P1P2;
T_IG = T_IP2*T_P2G;


% Extract the rotation matrices from each homogeneous transformation
% matrix.
R_IP1 = T_IP1(1:3,1:3);
R_IP2 = T_IP2(1:3,1:3);
R_IB = T_IB(1:3,1:3);

% Extract the position vectors from each homogeneous transformation
% matrix.
r_I_IB = T_IB(1:3,4);
r_I_IP1 = T_IP1(1:3,4);
r_I_IP2 = T_IP2(1:3,4);

% Define the unit vectors around which each link rotates in the precedent
% coordinate frame.
n_B = [0 0 0]';
n_1 = [1 0 0]';
n_2 = [1 0 0]';

% Compute the end-effector position vector.
r_I_IG = T_IG(1:3,4);

% Compute the translational jacobian.
J_IG_p = [  cross(R_IB*n_B, r_I_IG - r_I_IB) ...
            cross(R_IP1*n_1, r_I_IG - r_I_IP1) ...
            cross(R_IP2*n_2,  r_I_IG - r_I_IP2) ...
            ]


%%
% Rx = @(phi) [1, 0,         0;
%     0, cos(phi), -sin(phi);
%     0, sin(phi),  cos(phi)];
% B_n1 = [1;0;0];
% B_n2 = Rx(q1)*[1;0;0];
% B_r2G = Rx(q2)*Rx(q1)*[0;l22;l21];
% B_r1G = Rx(q1)*[0;0;l11]+B_r2G;
% 
% I_J_BG = [zeros(3,1),cross(B_n1,B_r1G),cross(B_n2,B_r2G)];
% J_IG_p = [[0;1;0],zeros(3,1),zeros(3,1)]+I_J_BG;
end