function [ddq_arm, q_base_next] = Q2_forward_dynamics(gc, tau, params)
% Compute generalized acceleration of the system
%
% Inputs:
%   - tau       : Current joint torques (6x1)
%   - gc        : Current generalized coordinates (q, dq)
%   - params    : Struct with parameters
%   
% Output:
%   - ddq          : Generalized accelerations for the arm joints (3x1) for the current simulation step
%   - q_base_next  : Generalized coordinates for the base (3x1) for the next simulation step.
%

if any(isnan(gc.q))
    error('q contains NaN');
end
if any(isnan(gc.dq))
    error('dq contains NaN');
end
if any(isnan(tau))
    error('tau contains NaN');
end

%% Setup
q = gc.q;      % Generalized coordinates (6x1)
dq = gc.dq;    % Generalized velocities (6x1)

M_arm = M_arm_fun_solution(q);
b_arm = b_arm_fun_solution(q,dq);
g_arm = g_arm_fun_solution(q);


%% Compute generalized acceleration of the system

% TODO: Implement ddq_arm here
ddq_arm = zeros(3, 1);

% TODO: For this simulation we set the base as fixed (i.e. never moves from its initial state).
% How should we then set the next step values of the base generalized coordinates?
q_base_next = zeros(3,1);

end
