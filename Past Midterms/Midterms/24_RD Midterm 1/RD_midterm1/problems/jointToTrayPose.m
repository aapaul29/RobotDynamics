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

T_IT = T_IB*T_BH*T_HP1*T_P1P2*T_P2T;

end

