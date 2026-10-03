function [ q_dot_des ] = gripperToJointPrioritized(w_IE_des_I, q, params)
% Inputs:
%     w_IE_des_I: 3x1 vector of desired 2D twist of gripper frame {E} wrt
%                 inertial frame {I} in inertial frame {I}
%     q: a 4x1 vector of generalized coordinates
%     params: a struct of parameters
%
% Outputs:
%     q_dot_des: 4x1 desired generalized velocities

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

% Pseudo-inverse damping coefficient
lambda = 1e-2;

J1 = jointTo2DGripperJacobian_solution(q,params);
J1_pseud_inv = pseudoInverseMat_solution(J1, lambda); % 4x3
q1_des = J1_pseud_inv*w_IE_des_I;

N = eye(4)-(J1_pseud_inv)*J1;

J2 = [0 0 0 0;0 0 1 0; 0 0 0 1];
w2_des = [0;0;0];
temp = pseudoInverseMat_solution(J2*N, lambda);
q2_des = temp*(w2_des-J2*J1_pseud_inv*w_IE_des_I);
q_dot_des = q1_des+N*q2_des;
end