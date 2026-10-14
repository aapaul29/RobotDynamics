function [ Dq ] = kinematicStandupControl( q, p_hip_joint_des, params )
% Inputs:
%  q               : current joint angles (6x1)
%  p_hip_joint_des : desired hip pose in inertial frame (3x1)
%  params          : a struct of parameters

% Output:
% Dq               : generalized coordinates velocity command (6x1)

% Proportional controller gain
K_p = 1;

% Pseudo_inverse damping coefficient
lambda = 1e-2;

% Generalized coordinates
q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);
q4 = q(5);
q5 = q(6);
l0 = params.l0;
l1 = params.l1;

% Implement your solution here...
% Current base position
r_B_curr = [q0; q1; q2];
% Desired Base posiiton
R = [cos(q2), -sin(q2), 0;
    sin(q2), cos(q2), 0;
    0, 0, 1];
r_B_des = p_hip_joint_des + [l0; l1; 0];
%Base error
r_B_err = r_B_des - r_B_curr;
% Base Jacobian
J_B = jointToBaseJacobian_solution(q, params);
% Tray Jacobian
J_T = jointToTrayJacobian_solution(q, params);
% Task Jacobian
J_task = [J_B;
    J_T(3, :);
    0, 0, 0, 0, 0, 1];
%Pseudo Inverse
J_task_psinv = pseudoInverseMat_solution(J_task, lambda);


Dq = J_task_psinv * [K_p * r_B_err; 0; 0];

end
