function [ J_IG_p ] = jointToPositionJacobian(q, params)
% Inputs:
%     q: a 3x1 vector of generalized coordinates
%     params: a struct of parameters
%
% Outputs:
%     J_IG_p: position Jacobian of gripper in inertia frame (3,3)

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

% Normal Vectors of joints in frame B
B_n_P1 = [1; 0; 0];
B_n_P2 = [1; 0; 0];

%Compute B_r_12
P1_r_12 = [0; 0; l11];

C_B1 = [1, 0, 0;
    0, cos(q1), -sin(q1);
    0, sin(q1), cos(q1)];

B_r_12 = C_B1 * P1_r_12;

% Compute B_r_2G
P2_r_2G = [0; l22; l21];

C_B2 = [1, 0, 0;
    0, cos(q1 + q2), -sin(q1 + q2);
    0, sin(q1 + q2), cos(q1 + q2)];

B_r_2G = C_B2 * P2_r_2G;

% Compute B_r_1G
B_r_1G = B_r_12 + B_r_2G;

J_BG = [zeros(3, 1), cross(B_n_P1, B_r_1G), cross(B_n_P2, B_r_2G)];

J_IB = [[0; 1; 0], zeros(3, 1), zeros(3, 1)];

% Implement your solution here...
J_IG_p = J_IB + J_BG;

end
