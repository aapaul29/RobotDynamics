function [p_des] = gripperToObject( q, pO, params )
% Inputs:
%  q            : current joint angles (3x1)
%  pO           : detected object pose in sensor frame (3x1)
%  params       : a struct of parameters

% Output:
%  p_des        : desired gripper pose in inertia frame (3x1)

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

% Object pose (in sensor frame)
pO_y = pO(1);
pO_z = pO(2);
pO_psi = pO(3);

% Implement your solution here ...
% I to B
C_IB = eye(3);
r_IB = [0; q0; l02];
T_IB = [C_IB, r_IB;
    zeros(1, 3), 1];

% B to S
C_BS = [1, 0, 0;
    0, cos(-pi + alpha), sin(-pi + alpha);
    0, sin(-pi + alpha), cos(-pi + alpha)];
r_BS = [0; l01 + l03 + l04; -l04 *cot(alpha)];
T_BS = [C_BS, r_BS;
    zeros(1, 3), 1];

% S to O
C_SO = [1, 0, 0;
    0, cos(pO_psi), -sin(pO_psi);
    0, sin(pO_psi), cos(pO_psi)];
r_SO = [0; pO_y; pO_z];
T_SO = [C_SO, r_SO;
    zeros(1, 3), 1];

% O to G
C_OG = [1, 0, 0;
    0, -1, 0;
    0, 0, -1];
r_OG = [0; 0; 0];
T_OG = [C_OG, r_OG;
    zeros(1, 3), 1];

T_IG = T_IB * T_BS * T_SO * T_OG;

% Find p_des
p_des = zeros(3, 1);
p_des(1: 2) = T_IG(2:3, 4);
p_des(3) = atan2(T_IG(3, 2), T_IG(2, 2));
end
