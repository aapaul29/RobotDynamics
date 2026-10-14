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
% From I to B
C_IB = [-1, 0, 0;
    0, 1, 0;
    0, 0, -1];
r_IB = [l1; 0; l0];
T_IB = [C_IB, r_IB;
    zeros(1, 3), 1];

% From B to P0
C_B0 = eye(3);
r_B0 = [0; 0; l2 + q0];
T_B0 = [C_B0, r_B0;
    zeros(1, 3), 1];

% From P0 to P1
C_01 = [cos(q1), 0, sin(q1);
    0, 1, 0;
    -sin(q1), 0, cos(q1)];
r_01 = [0; 0; l3];
T_01 = [C_01, r_01;
    zeros(1, 3), 1];

% From P1 to P2
C_12 = [cos(q2), 0, sin(q2);
    0, 1, 0;
    -sin(q2), 0, cos(q2)];
r_12 = [0; 0; l4];
T_12 = [C_12, r_12;
    zeros(1, 3), 1];

% From P2 to P3
C_23 = [cos(q3), 0, sin(q3);
    0, 1, 0;
    -sin(q3), 0, cos(q3)];
r_23 = [0; 0; l5];
T_23 = [C_23, r_23;
    zeros(1, 3), 1];

% From P3 to E
C_3E = eye(3);
r_3E = [0; 0; l6];
T_3E = [C_3E, r_3E;
    zeros(1, 3), 1];


T_IE = T_IB * T_B0 * T_01 * T_12 * T_23 * T_3E;

end
