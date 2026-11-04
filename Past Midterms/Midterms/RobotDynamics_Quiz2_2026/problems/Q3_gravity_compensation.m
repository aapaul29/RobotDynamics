function [ tau ] = Q3_gravity_compensation(params, gc, q_des, dq_des)
% Joint space PD controller with gravity compensation.
%
% Inputs:
%   - params    : struct with parameters
%   - gc        : Current generalized coordinates (q, dq)
%   - q_des     : the desired joint positions (4x1)
%   - dq_des    : the desired joint velocities (4x1)
% Output:
%   - tau       : computed control torque per joint (4x1)
%
%% Setup
q = gc.q;      % Generalized coordinates (4x1)
dq = gc.dq;    % Generalized velocities (4x1)

M = M_fun_solution(q);% Mass matrix
b = b_fun_solution(q, dq); % Nonlinear term
g = g_fun_solution(q); % Gravity term

% Gains !!! Please do not modify these gains !!!
Kp = params.Kp_joint; % P gain matrix for joints (4x4 diagonal matrix)
Kd = params.Kd_joint; % D gain matrix for joints (4x4 diagonal matrix)


%% Compute torque
tau = Kp*(q_des - q) + Kd*(dq_des - dq) + g; % TODO: Implement tau here

end
