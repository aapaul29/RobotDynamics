function tau = Q4_inverse_dynamics_control(params, gc, I_r_IDd, I_v_Dd, I_a_Dd, C_IDd)
% Inverse dynamics controller tracking a desired drink motion.
%
% Inputs:
%   - params    : struct with parameters
%   - gc        : Current generalized coordinates (q, dq)
%   - I_r_IDd   : desired position of the drink in the inertial frame (3x1)
%   - I_v_Dd    : desired linear velocity of the drink in the inertial frame (3x1)
%   - I_a_Dd    : desired linear acceleration of the drink in the inertial frame (3x1)
%   - C_IDd     : desired orientation of the drink as a rotation matrix (3x3)
% Output:
%   - tau       : computed generalized forces (4x1), tau(4) is ignored (fork actuator off)
%
%% Setup
q = gc.q;      % Generalized coordinates (4x1)
dq = gc.dq;    % Generalized velocities (4x1)

M = M_fun_solution(q);     % Mass matrix
b = b_fun_solution(q, dq); % Nonlinear term
g = g_fun_solution(q);     % Gravity term
I_g_acc = params.I_g_acc;

% Kinematics and Jacobians of the drink frame {D}
T_ID = T_ID_fun(q);                 % Homogeneous transformation from {D} to {I} (4x4)
I_Jp_D = I_Jp_D_fun(q);             % Positional Jacobian of the drink (3x4)
I_Jr_D = I_Jr_D_fun(q);             % Rotational Jacobian of the drink (3x4)
I_dJp_D = I_dJp_D_fun(q, dq);       % Time derivative of the positional Jacobian (3x4)
I_dJr_D = I_dJr_D_fun(q, dq);       % Time derivative of the rotational Jacobian (3x4)

% Gains !!! Please do not modify these gains !!!
Kp = params.Kp_task; % P gain matrix for the drink pose (6x6 diagonal matrix)
Kd = params.Kd_task; % D gain matrix for the drink velocity (6x6 diagonal matrix)

% Mass of the full glass, not part of M, b and g
m_glass = params.m_glass;

%Task Jacobian
J = [I_Jp_D; I_Jr_D];
dJ = [I_dJp_D; I_dJr_D];

%Errors
r_err = I_r_IDd - T_ID(1:3,4);
C_err = C_IDd * T_ID(1:3, 1:3)';
orientation_error = rotMatToRotVec(C_err);
dchi_err = [r_err; orientation_error];
%Velocity error
w_err = [I_v_Dd; zeros(3, 1)] - J*dq;
%desired w
w_des = Kp*dchi_err + Kd * w_err;
%desired ddq
ddq = pseudoInverseMat(J) * (w_des - dJ * dq);
%External force
F_ext = [0;0;-m_glass*I_g_acc;0];

%% Compute torque
tau = M*ddq + b + g + J'*F_ext; % TODO: Implement tau here

end
