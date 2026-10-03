function [ J_IT_p ] = jointToPositionJacobian(q, params)
% Inputs:
%     q: a 6x1 vector of generalized coordinates
%     params: a struct of parameters
%
% Outputs:
%     J_IT_p: position Jacobian of tray frame {E} wrt inertia frame {I}
%     expressed in inertial frame {I}

% link lengths (meters)
l0 = params.l0;
l1 = params.l1;
l2 = params.l2;
l3 = params.l3;

% Generalized coordinates
q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);
q4 = q(5);
q5 = q(6);

% Implement your solution here...
% Implement your solution here ...
rz = @(theta) [cos(theta), -sin(theta), 0;% RADIANS
    sin(theta), cos(theta), 0;% RADIANS
    0, 0, 1];% RADIANS


C_IB = rz(q2);
t_IB = [q0;q1;0];
T_IB = [C_IB,t_IB;0 0 0 1];

C_BH =eye(3);
t_BH = [-l0;-l1;0];
T_BH = [C_BH,t_BH;0 0 0 1];

C_HP1 = rz(-pi/2+q3);
t_HP1 = [2*l0;0;0];
T_HP1 = [C_HP1,t_HP1;0 0 0 1];

C_P1P2 = rz(q4);
t_P1P2 = [l2;0;0];
T_P1P2 = [C_P1P2,t_P1P2;0 0 0 1];

C_P2T = rz(q5);
t_P2T = [l3;0;0];
T_P2T = [C_P2T,t_P2T;0 0 0 1];


T_IH = T_IB*T_BH;
T_IP1= T_IH*T_HP1;
T_IP2 = T_IP1*T_P1P2;
T_IT = T_IP2*T_P2T;


% Extract the rotation matrices from each homogeneous transformation
% matrix.
R_IB = T_IB(1:3,1:3);
R_IH = T_IH(1:3,1:3);
R_IP1 = T_IP1(1:3,1:3);
R_IP2 = T_IP2(1:3,1:3);
R_IT = T_IT(1:3,1:3);

% Extract the position vectors from each homogeneous transformation
% matrix.
r_I_IB = T_IB(1:3,4);
r_I_IH = T_IH(1:3,4);
r_I_IP1 = T_IP1(1:3,4);
r_I_IP2 = T_IP2(1:3,4);
r_I_IT = T_IT(1:3,4);
% Define the unit vectors around which each link rotates in the precedent
% coordinate frame.
n_2 = [0 0 1]';%q2
n_3 = [0 0 1]';%q3
n_4 = [0 0 1]';%q4



% Compute the translational jacobian.
J_P = [[0;0;0], ...
    [0;0;0]...
    cross(R_IB*n_2, r_I_IT - r_I_IB) ...
    cross(R_IP1*n_3, r_I_IT - r_I_IP1) ...
    cross(R_IP2*n_4, r_I_IT - r_I_IP2) ...
    [0;0;0]];

J_IT_p = J_P+[[1;0;0],[0;1;0], zeros(3,4)];

end

