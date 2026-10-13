function [ Dq ] = kinematicTrajectoryControl( q, p_base_des, params )
% Inputs:
%  q             : current joint angles (6x1)
%  p_base_des    : desired 2D base pose (x,y,q1) (3x1)
%  params        : a struct of parameters
%
% Output:
% Dq             : generalized coordinates velocity command (6x1)

% Proportional controller gain
K_p = 10;

% Pseudo_inverse damping coefficient
lambda = 1e-2;

% Gerneralized coordinates
q0  = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);
q4 = q(5);
q5 = q(6);

% Current Base position
r_curr = [q0; q1; q2];

% Base position Error
r_err = p_base_des - r_curr;

% Base Jacobian
J_IB = jointToBaseJacobian_solution(q, params);

% Tray Jacobian
J_IT = jointToTrayJacobian_solution(q, params);

% Task Jacobian
J_task = [J_IB; J_IT];

%pseudo Inverse
J_task_pinv = pseudoInverseMat_solution(J_task, lambda);

Dq = J_task_pinv * [K_p * r_err; 0; 0; 0];



end
