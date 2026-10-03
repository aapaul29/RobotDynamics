% params: a struct of parameters

% Link lengths (meters)
l0 = params.l0;
l1 = params.l1;
l2 = params.l2;
l31 = params.l31;
l32 = params.l32;

% Joint positions
%

rx = @(theta) [1, 0, 0;
               0, cos(theta), -sin(theta);
               0, sin(theta), cos(theta)];
ry = @(theta) [cos(theta), 0, sin(theta);
               0, 1, 0;
               -sin(theta), 0, cos(theta)];
rz = @(theta) [cos(theta), -sin(theta), 0;
               sin(theta), cos(theta), 0;
               0, 0, 1];
C_IO = rx(pi/2)*ry(pi/2);
trasl_IO = [0;0;l0];
T_IO = [C_IO,trasl_IO;[0 0 0 1]];

C_O1 = ry(q1);
trasl_O1 = [0;l1;0];
T_O1 = [C_O1,trasl_O1;[0 0 0 1]];

C_12 = rz(q2);
trasl_12 = [0;0;0];
T_12 = [C_12,trasl_12;[0 0 0 1]];

C_23 = rz(q3);
trasl_23 = [l2;0;0];
T_23 = [C_23,trasl_23;[0 0 0 1]];

C_3E = [eye(3)];
trasl_3E =[l31+l32;0;0];
T_3E = [C_3E,trasl_3E;[0 0 0 1]];

T_IE = T_IO*T_O1*T_12*T_23*T_3E;