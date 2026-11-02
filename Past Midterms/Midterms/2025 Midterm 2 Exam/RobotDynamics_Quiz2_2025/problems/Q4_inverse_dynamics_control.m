function [tau] = Q4_inverse_dynamics_control( params, gc, kin, I_r_IEd, I_v_Ed, C_IEd, I_F_Ezd)
% Inverse dynamics controller tracking a desired end effector (EE) motion
% with a PD stabilizing feedback terms.
%
% Inputs:
%   - params    : struct with parameters
%   - gc        : Current generalized coordinates (q, dq)
%   - kin       : struct with kinematics
%   - I_r_IEd   : the desired position (3x1) of the tray w.r.t. the inertial frame expressed in the inertial frame.
%   - C_IEd     : the desired orientation of the tray as a rotation matrix (3x3)
%   - I_v_Ed    : the desired linear velocity (3x1) of the tray expressed in the inertial frame.
%   - I_F_Ezd   : the desired compensating force in the z axis of the inertial frame.
% Output:
%   - tau       : computed control torque per joint (8x1)
%

%% Setup
q = gc.q;      % Generalized coordinates (8x1)
dq = gc.dq;    % Generalized velocities (8x1)

M = M_fun_solution(q); % Mass matrix
b = b_fun_solution(q, dq); % Nonlinear term
g = g_fun_solution(q); % Gravity term

% Find jacobians, positions and orientation based on the current
I_Jp_EE_left = I_Jp_EE_left_fun(q); % Positional Jacobian of the left ee
I_Jr_EE_left = I_Jr_EE_left_fun(q); % Rotational Jacobian of the left ee
I_dJp_EE_left = I_dJp_EE_left_fun(q, dq); % Time derivative of the position Jacobian of the left ee
I_dJr_EE_left = I_dJr_EE_left_fun(q, dq); % Time derivative of the Rotational Jacobian of the left ee

% Geometrical Jacobian
I_J_EE_left = [I_Jp_EE_left; I_Jr_EE_left];
I_dJ_EE_left = [I_dJp_EE_left; I_dJr_EE_left];

% Kinematics
T_IEE_left = eval(subs(kin.T_I8, {'q0' 'q1' 'q2' 'q3' 'q4' 'q5' 'q6' 'q7'}, {gc.q(1) gc.q(2) gc.q(3) gc.q(4) gc.q(5) gc.q(6) gc.q(7) gc.q(8)}));
R_IEE_left = eval(subs(kin.R_I8, {'q0' 'q1' 'q2' 'q3' 'q4' 'q5' 'q6' 'q7'}, {gc.q(1) gc.q(2) gc.q(3) gc.q(4) gc.q(5) gc.q(6) gc.q(7) gc.q(8)}));
I_r_IEE_left = eval(subs(kin.I_r_I8, {'q0' 'q1' 'q2' 'q3' 'q4' 'q5' 'q6' 'q7'}, {gc.q(1) gc.q(2) gc.q(3) gc.q(4) gc.q(5) gc.q(6) gc.q(7) gc.q(8)}));

%% Compute control torque

% Gains !!! Please do not modify these gains !!!
kp = params.kp_task; % P gain matrix for object position (8x8 diagonal matrix)
kd = params.kd_task; % D gain matrix for object velocity  (8x8 diagonal matrix)

% TODO
%Errors
r_err = I_r_IEd - I_r_IEE_left;
orientation_error = rotMatToRotVec(C_IEd * R_IEE_left');
dchi_err = [r_err; orientation_error];
%velocity error (no desired angular velocity given -> zero)
dw_err = [I_v_Ed; zeros(3,1)] - I_J_EE_left * dq;
%desired acc
dw_des = kp * dchi_err + kd * dw_err;
%desired ddq
ddq = pseudoInverseMat(I_J_EE_left) * (dw_des - I_dJ_EE_left * dq);

%external force acting on the left ee (measured) -> cancel it
I_F_E = [0; 0; I_F_Ezd];

%inverse dynamics
tau = M * ddq + b + g - I_Jp_EE_left' * I_F_E;

%right arm (q3, q5, q7) is broken -> no torques
tau([4 6 8]) = 0;

end