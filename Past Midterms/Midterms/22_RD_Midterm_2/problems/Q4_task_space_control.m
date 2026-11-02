function [tau, dyn] = Q4_task_space_control( params, gc, kin, I_r_IGd, I_v_Gd, C_IGd, I_F_Gy)
% Task-space inverse dynamics controller tracking a desired end-effector motion
% with a PD stabilizing feedback terms.
%
% Inputs:
%   - params    : struct with parameters
%   - gc        : Current generalized coordinates (q, dq)
%   - kin       : struct with kinematics
%   - I_r_IGd   : the desired position (3x1) of the gripper w.r.t. the inertial frame expressed in the inertial frame.
%   - I_v_Gd    : the desired linear velocity (3x1) of the gripper in the inertial frame.
%   - C_IGd     : the desired orientation of the gripper as a rotation matrix (3x3)
%   - I_F_Gy    : the desired force in y direction.
% Output:
%   - tau       : computed control torque per joint (3x1)
%
%% Setup
q = gc.q;      % Generalized coordinates (3x1)
dq = gc.dq;    % Generalized velocities (3x1)

M = M_fun_solution(q); % Mass matrix
b = b_fun_solution(q, dq); % Nonlinear term
g = g_fun_solution(q); % Gravity term

% Find jacobians, positions and orientation based on the current
I_Jp_G = I_Jp_G_fun(q); % Positional Jacobian of end effector
I_Jr_G = I_Jr_G_fun(q); % Rotational Jacobian of end effector
I_dJp_G = I_dJp_G_fun(q, dq); % Time derivative of the position Jacobian of the end-effector (3x3)
I_dJr_G = I_dJr_G_fun(q, dq); % Time derivative of the Rotational Jacobian of the end-effector (3x3)

% Geometrical Jacobian
I_J_G = [I_Jp_G; I_Jr_G];
I_dJ_G = [I_dJp_G; I_dJr_G];

% Kinematics
T_IG = eval(subs(kin.T_IG, {'q0' 'q1' 'q2'}, {gc.q(1) gc.q(2) gc.q(3)}));
I_r_IG = eval(subs(kin.I_r_IG, {'q0' 'q1' 'q2'}, {gc.q(1) gc.q(2) gc.q(3)}));

% Desired Force
I_F_G = zeros(6,1); % Desired end-effector force in inertial frame
I_F_G(2) = I_F_Gy; % Normal force

%% Project the joint-space dynamics to the task space
% Note: use pseudoInverseMat() function for lambda for stability
% TODO: Implement end-effector dynamics
JMinv = I_J_G / M;
lambda = pseudoInverseMat(JMinv*I_J_G');
mu = lambda * JMinv *b - lambda * I_dJ_G * dq;
p =  lambda * JMinv * g;

%% Compute torque
% TODO: Implement task-space
% Note: desired angular velocity & acceleration are zero.

% Gains !!! Please do not modify these gains !!!
kp = params.kp_task; % P gain matrix for gripper position (6x6 diagonal matrix)
kd = params.kd_task; % D gain matrix for gripper velocity  (6x6 diagonal matrix)
% Orientation error
C_err = C_IGd * T_IG(1:3, 1:3)';
orientation_error = rotMatToRotVec(C_err);
%Parametrization error
chi_err = [I_r_IGd - I_r_IG; orientation_error];
dchi_err = [I_v_Gd;zeros(3, 1)] - I_J_G * dq;
dw_e = [zeros(3,1); zeros(3,1)] + kp * chi_err + kd * dchi_err;
%Selection matrices
sigma_p = diag([1.0, 0.0, 1.0]);
sigma_r = diag([1.0,1.0,1.0]);
SM = [sigma_p, zeros(3);
    zeros(3), sigma_r];
SF = [eye(3) - sigma_p, zeros(3);
    zeros(3), (eye(3) - sigma_r)];
%desired Force
I_F_des = lambda * SM * dw_e + SF * I_F_G + mu + p;

tau = I_J_G' * I_F_des;  % TODO: Implement tau

%% Return for evaluation
dyn.lambda =lambda;
dyn.mu = mu;
dyn.p = p;
end
