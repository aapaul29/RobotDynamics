function [ T_IT ] = jointToTrayPose(q, params)
% Inputs:
%     q: a 6x1 vector of generalized coordinates (6, 1).
%     params: a struct of parameters.
%
% Outputs:
%     T_IT: Homogenous transform from tray frame {T} to inertial
%     frame {I}

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

% Implement your solution here ...
% Compute T_IB
C_IB = [cos(q2), -sin(q2), 0;
    sin(q2), cos(q2), 0;
    0, 0, 1];
r_IB = [q0; q1; 0];
T_IB = [C_IB, r_IB;
    zeros(1, 3), 1];

% Compute T_B1
C_B1 = [cos(-q3), -sin(-q3), 0;
    sin(-q3), cos(-q3), 0;
    0, 0, 1];
r_B1 = [l0; -l1; 0];
T_B1 = [C_B1, r_B1;
    zeros(1, 3), 1];

% Compute T_12
C_12 = [cos(q4), -sin(q4), 0;
    sin(q4), cos(q4), 0;
    0, 0, 1];
r_12 = [l2; 0; 0];
T_12 = [C_12, r_12;
    zeros(1, 3), 1];

% Compute T_2T
C_2T = [cos(-q5), -sin(-q5), 0;
    sin(-q5), cos(-q5), 0;
    0, 0, 1];
r_2T = [l3; 0; 0];
T_2T = [C_2T, r_2T;
    zeros(1, 3), 1];

% Compute T_IT

T_IT = T_IB * T_B1 * T_12 * T_2T;

end
