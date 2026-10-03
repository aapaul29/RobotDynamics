function [ q_dot_cmd ] = kinematicTrajectoryControl( q, p_IO_des_I, w_IO_des_I, params )
% Inputs:
%  q             : 4x1 current joint angles
%  p_des         : 3x1 desired 2D object pose
%  w_des         : 3x1 desired 2D object twist
%  params        : a struct of parameters

% Output:
%  q_dot_cmd     : 4x1 joint velocities command

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
R_IE = T_IE(1:3,1:3);
p_IE_des_I = p_IO_des_I-R_IE*[0;l7;0];

% Pseudo-inverse damping coefficient
lambda = 1e-3;

% Implement your solution here...

kp = 1e-9;
r_IE_nominal = jointTo2DGripperPosition_solution(q, params);


dr_IE_I = r_IE_nominal-p_IE_des_I(1:2);
dr_IE_I(end+1) = p_IO_des_I(end);
J = jointTo2DGripperJacobian_solution(q,params);
J_ps = pseudoInverseMat_solution(J, lambda);

q_dot_cmd = J_ps*(w_IO_des_I+kp*dr_IE_I);

end