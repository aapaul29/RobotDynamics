function [ T_IE ] = jointToEndeffectorPose( q, params )
% q: a 3x1 vector of generalized coordinates
% params: a struct of parameters
l0 = params.l0;
l1 = params.l1;
l2 = params.l2;
l31 = params.l31;
l32 = params.l32;
l4 = params.l4;
l5 = params.l5;

% Get Joint angles
q1 = q(1);
q2 = q(2);
q3 = q(3);

% Compute T_I0
C_I0 = [cos(q1), 0, sin(q1);
    0, 1, 0;
    -sin(q1), 0, cos(q1)];

I_r_I0 = [0, 0, l0]';

T_I0 = [C_I0, I_r_I0;
    zeros(1, 3), 1];

% Compute T_01
C_01 = [1, 0, 0;
    0, 1, 0;
    0, 0, 1];

r_01 = [0, l1, 0]';

T_01 = [C_01, r_01;
    zeros(1, 3), 1];

% Compute T_12
C_12 = [cos(q2), -sin(q2), 0;
    sin(q2), cos(q2), 0;
    0, 0, 1];

r_12 = [0, 0, 0]';

T_12 = [C_12, r_12;
    zeros(1, 3), 0];

% Compute T_23
C_23 = [cos(q3), -sin(q3), 0;
    sin(q3), cos(q3), 0;
    0, 0, 1];

r_23 =[l2, 0, 0]';

T_23 = [C_23, r_23;
    zeros(1, 3), 1];

% Compute T_3E
C_3E = [1, 0, 0;
    0, 1, 0;
    0, 0, 1];

r_3E = [l31 + l32, 0, 0]';

T_3E = [C_3E, r_3E;
    zeros(1, 3), 1];


T_IE = T_I0 * T_01 * T_12 * T_23 * T_3E;

end