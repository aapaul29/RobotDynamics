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

%Compute T_IB
C_IB = [1, 0, 0;
    0, 1, 0;
    0, 0, 1];

r_IB = [0; q0; l02];

T_IB = [C_IB, r_IB;
    zeros(1, 3), 1];

% Compute T_BP1
C_BP1 = [1, 0, 0;
    0, cos(q1), -sin(q1);
    0, sin(q1), cos(q1)];

r_BP1 = [0; l01; 0];

T_BP1 = [C_BP1, r_BP1;
    zeros(1, 3), 1];

% Compute T_P1P2
C_P1P2 = [1, 0, 0;
    0, cos(q2), -sin(q2);
    0, sin(q2), cos(q2)];

r_P1P2 = [0; 0; l11];

T_P1P2 = [C_P1P2, r_P1P2;
    zeros(1, 3), 1];

% Compute T_P2G
C_P2G = [1, 0, 0;
    0, 0, 1;
    0, -1, 0];

r_P2G = [0; l22; l21];

T_P2G = [C_P2G, r_P2G;
    zeros(1, 3), 1];

% Compute T_IG:
T_IG = T_IB * T_BP1 * T_P1P2 * T_P2G;
end
