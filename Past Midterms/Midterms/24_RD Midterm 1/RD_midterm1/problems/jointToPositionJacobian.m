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
% Compute T_IB
C_IB = [cos(q2), -sin(q2), 0;
    sin(q2), cos(q2), 0;
    0, 0, 1];
r_IB = [q0; q1; 0];
T_IB = [C_IB, r_IB;
    zeros(1, 3), 1];

% Compute T_B1
C_B1 = [cos(-q3), sin(-q3), 0;
    sin(-q3), cos(-q3), 0;
    0, 0, 1];
r_B1 = [l0; -l1; 0];
T_B1 = [C_B1, r_B1;
    zeros(1, 3), 1];

% Compute T_12
C_12 = [cos(q4), sin(q4), 0;
    sin(-q4), cos(q4), 0;
    0, 0, 1];
r_12 = [l2; 0; 0];
T_12 = [C_12, r_12;
    zeros(1, 3), 1];

% Compute T_2T
C_2T = [cos(-q5), sin(-q5), 0;
    sin(-q5), cos(-q5), 0;
    0, 0, 1];
r_2T = [l3; 0; 0];
T_2T = [C_2T, r_2T;
    zeros(1, 3), 1];

% Compute T_IT
T_IT = T_IB * T_B1 * T_12 * T_2T;


% Normal Vectors
n0 = [1; 0; 0];
n1 = [0; 1; 0];
n2 = [0; 0; 1];
n3 = [0; 0; 1];
n4 = [0; 0; 1];
n5 = [0; 0; 1];

% joint B
T_BT = T_B1 * T_12 * T_2T;
r_BT = T_BT(1:3, 4);
I_r_BT = C_IB * r_BT;

% joint 1
T_1T = T_12 * T_2T;
C_I1 = C_IB * C_B1;
r_1T = T_1T(1:3, 4);
I_r_1T = C_I1 * r_1T;

% Joint 2
C_I2 = C_I1 * C_12;
I_r_2T = C_I2 * r_2T;

J_IT_p = [n0, n1, cross(n2, I_r_BT), cross(n3, I_r_1T), cross(n4, I_r_2T), [0; 0; 0]];

end
