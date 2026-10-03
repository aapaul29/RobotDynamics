function [ J_3C_3 ] = point3ToCameraGeometricJacobian(q, params)
% Inputs:
%     q: a 3x1 vector of generalized coordinates
%     params: a struct of parameters
% Output:
%     J_3C_3: geometric Jacobian from point O_3 to point C, expressed in
%     frame {3}

theta = params.theta;
l31 = params.l31;
l4 = params.l4;
l5 = params.l5;
l0 = params.l0;
l1 = params.l1;
l2= params.l2;
l32 = params.l32;
q1 = q(1);
q2 = q(2);
q3 = q(3);
% your implementation here...


rx = @(theta) [1, 0, 0;
               0, cos(theta), -sin(theta);
               0, sin(theta), cos(theta)];
ry = @(theta) [cos(theta), 0, sin(theta);
               0, 1, 0;
               -sin(theta), 0, cos(theta)];
rz = @(theta) [cos(theta), -sin(theta), 0;
               sin(theta), cos(theta), 0;
               0, 0, 1];


C_O1 = ry(q1);
trasl_O1 = [0;l1;0];
T_O1 = [C_O1,trasl_O1;[0 0 0 1]];

C_12 = rz(q2);
trasl_12 = [0;0;0];
T_12 = [C_12,trasl_12;[0 0 0 1]];

C_23 = rz(q3);
trasl_23 = [l2;0;0];
T_23 = [C_23,trasl_23;[0 0 0 1]];

T_O3 = T_O1*T_12*T_23;
C_O3 = T_O3(1:3,1:3);
%3_n_1 = C_30*0_n_1 
O_n_1 = [0;1;0];
n_1_exp_3 = C_O3'*O_n_1;%3_n_1

T_13 = T_12*T_23;
C_13 = T_13(1:3,1:3);
%3_n_2 = C_31*1_n_2
n_2_exp_1 = [0;0;1];
n_2_exp_3 = C_13'*n_2_exp_1;%3_n_2

%3_n_3
n_3_exp_3 = [0;0;1];

r_OC_3 = [l31+l4*cos(theta)+l5*cos(pi/2-theta); l4*sin(theta)-l5*sin(pi/2-theta);0];


J_3C_3_pos = [cross(n_1_exp_3,r_OC_3),cross(n_2_exp_3,r_OC_3),cross(n_3_exp_3,r_OC_3)];


J_3C_3_rot = [zeros(3,3)];



J_3C_3 = [J_3C_3_pos;J_3C_3_rot];

end