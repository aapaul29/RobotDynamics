function [ J_3C_3 ] = point3ToCameraGeometricJacobian(q, params)
% Inputs:
%     q: a 3x1 vector of generalized coordinates
%     params: a struct of parameters
% Output:
%     J_3C_3: geometric Jacobian from point O_3 to point C, expressed in
%     frame {3}

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
theta = params.theta;
% your implementation here...
% ROtation Matrix C_3C
C_3C = [cos(theta - pi / 2), -sin(theta - pi / 2), 0;
    sin(theta - pi / 2), cos(theta - pi / 2), 0;
    0, 0, 1];

% Position of C in 3 frame
r_3C = [l31; 0; 0] + C_3C * [l5; l4; 0];

% Rotation Matrix C_13
C_13 = [cos(q2 + q3), -sin(q2 + q3), 0;
    sin(q2 + q3), cos(q2 + q3), 0;
    0, 0, 1];

% Normal Vectors of joint rotations in 3 frame
n1_3 = C_13' * [0; 1; 0];
n2_3 = [0; 0; 1];
n3_3 = [0; 0; 1];

% Jacobian
J_3C_3 = [cross(n1_3, r_3C), cross(n2_3, r_3C), cross(n3_3, r_3C)];

end