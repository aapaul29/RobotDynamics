function [tau] = Q5_task_space_control( params, gc, I_r_ICd, I_v_Cd, I_a_Cd)
% Task-space inverse dynamics controller tracking a desired end-effector motion
% with a PD stabilizing feedback terms.
%
% Inputs:
%   - params    : struct with parameters
%   - gc        : Current generalized coordinates (q, dq)
%   - I_r_ICd   : the desired position (3x1) of the camera w.r.t. the inertial frame expressed in the inertial frame.
%   - I_v_Cd    : the desired linear velocity (3x1) of the camera in the inertial frame.
%   - I_a_Cd    : the desired linear acceleration (3x1) of the camera in the inertial frame.
% Output:
%   - tau       : computed control torque per joint (3x1)
%
%% Setup
q = gc.q;      % Generalized coordinates (3x1)
dq = gc.dq;    % Generalized velocities (3x1)

M = M_fun_solution(q); % Mass matrix
b = b_fun_solution(q, dq); % Nonlinear term
g = g_fun_solution(q); % Gravity term

% Gains !!! Please do not modify these gains !!!
kp = params.kp_Q5; % P gain matrix for camera position (3x3 diagonal matrix)
kd = params.kd_Q5; % D gain matrix for camera velocity  (3x3 diagonal matrix)

% Find jacobians, positions and orientation based on the current
I_Jp_C = I_Jp_C_fun(q); % Positional Jacobian of end effector
I_Jr_C = I_Jr_C_fun(q); % Rotational Jacobian of end effector
I_dJp_C = I_dJp_C_fun(q, dq); % Time derivative of the position Jacobian of the end-effector (3x3)
I_dJr_C = I_dJr_C_fun(q, dq); % Time derivative of the Rotational Jacobian of the end-effector (3x3)

% Geometrical Jacobian
I_J_C = [I_Jp_C; I_Jr_C];
I_dJ_C = [I_dJp_C; I_dJr_C];

T_IC = T_IC_fun(q); % Homogeneous transformation from frame C to frame I
I_r_IC = T_IC(1:3, 4);

%% Compute torque

% Step 1: Choose the task Jacobian.
% The task only specifies linear camera motion, so we keep the 3 position
% rows and ignore the rotational rows in I_J_C. J maps joint velocities to
% the camera's linear velocity: v = J * dq.
J = I_Jp_C;      % (3x3)
dJ = I_dJp_C;    % (3x3)

% Step 2: Compute the acceleration we want the camera to have.
% Feedforward (I_a_Cd) makes it follow the planned motion, and the PD terms
% correct any position and velocity errors. The current camera velocity
% comes from the Jacobian.
I_v_C = J * dq;
I_a_C_cmd = I_a_Cd + kp * (I_r_ICd - I_r_IC) + kd * (I_v_Cd - I_v_C);

% Step 3: Project the dynamics into task space.
% Differentiating v = J * dq gives a = J * ddq + dJ * dq. Substituting
% ddq = M^-1 * (tau - b - g) with tau = J' * F gives
%   Lambda * a + mu + p = F
% which is M * ddq + b + g = tau, written for the camera instead of the joints.
%
% Lambda: end-effector inertia, the mass the camera "feels" (3x3).
% J / M means J * inv(M), computed without forming the inverse. The damped
% pseudo-inverse keeps Lambda bounded when J * inv(M) * J' is near singular.
damping = 1e-3;
Lambda = pseudoInverseMat(J / M * J', damping);

% mu: Coriolis and centrifugal terms seen at the camera (3x1).
% The -dJ * dq part removes the acceleration the camera already has from
% the joints' motion.
mu = Lambda * (J / M * b - dJ * dq);

% p: gravity seen at the camera (3x1).
p = Lambda * (J / M * g);

% Step 4: Compute the force at the camera, then map it to joint torques.
% F = Lambda * a + mu + p is the force needed for the commanded acceleration.
% J' maps a Cartesian force to joint torques, as in virtual work.
I_F_C = Lambda * I_a_C_cmd + mu + p;
tau = J' * I_F_C;

end
