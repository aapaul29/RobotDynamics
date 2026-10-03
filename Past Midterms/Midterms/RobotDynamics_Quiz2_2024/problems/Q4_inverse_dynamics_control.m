function [tau] = Q4_inverse_dynamics_control(params, gc, kin, I_r_ITd, C_ITd, I_v_Td, I_F_Txd)
% Inverse dynamics controller tracking a desired tray motion
% with a PD stabilizing feedback terms.
%
% Inputs:
%   - params    : struct with parameters
%   - gc        : Current generalized coordinates (q, dq)
%   - kin       : struct with kinematics
%   - I_r_ITd   : the desired position (3x1) of the tray w.r.t. the inertial frame expressed in the inertial frame.
%   - C_ITd     : the desired orientation of the tray as a rotation matrix (3x3)
%   - I_v_Td    : the desired linear velocity (3x1) of the tray expressed in the inertial frame.
%   - I_F_Txd   : the desired compensating force in the x axis of the inertial frame.
% Output:
%   - tau       : computed control torque per joint (6x1)
%

%% Setup
q = gc.q;      % Generalized coordinates (6x1)
dq = gc.dq;    % Generalized velocities (6x1)

M = M_fun_solution(q); % Mass matrix
b = b_fun_solution(q, dq); % Nonlinear term
g = g_fun_solution(q); % Gravity term

% Find jacobians, positions and orientation based on the current
I_Jp_T = I_Jp_T_fun(q); % Positional Jacobian of the tray
I_Jr_T = I_Jr_T_fun(q); % Rotational Jacobian of the tray
I_dJp_T = I_dJp_T_fun(q, dq); % Time derivative of the position Jacobian of the tray (3x6)
I_dJr_T = I_dJr_T_fun(q, dq); % Time derivative of the Rotational Jacobian of the tray (3x6)

% Geometrical Jacobian
I_J_T = [I_Jp_T; I_Jr_T];
I_dJ_T = [I_dJp_T; I_dJr_T];

% Kinematics
T_IT = eval(subs(kin.T_IT, {'q0' 'q1' 'q2' 'q3' 'q4' 'q5'}, {gc.q(1) gc.q(2) gc.q(3) gc.q(4) gc.q(5) gc.q(6)}));
R_IT = eval(subs(kin.R_IT, {'q0' 'q1' 'q2' 'q3' 'q4' 'q5'}, {gc.q(1) gc.q(2) gc.q(3) gc.q(4) gc.q(5) gc.q(6)}));
I_r_IT = eval(subs(kin.I_r_IT, {'q0' 'q1' 'q2' 'q3' 'q4' 'q5'}, {gc.q(1) gc.q(2) gc.q(3) gc.q(4) gc.q(5) gc.q(6)}));

%% Compute control torque

% Gains !!! Please do not modify these gains !!!
kp = params.kp_task; % P gain matrix for object position (6x6 diagonal matrix)
kd = params.kd_task; % D gain matrix for object velocity  (6x6 diagonal matrix)

% TODO
tau = zeros(6,1);

end
