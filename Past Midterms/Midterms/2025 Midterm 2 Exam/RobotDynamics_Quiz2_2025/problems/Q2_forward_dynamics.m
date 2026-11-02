function ddq = Q2_forward_dynamics(gc, tau, params)
% Compute generalized acceleration of the system
%
% Inputs:
%   - tau       : Current joint torques (8x1)
%   - gc        : Current generalized coordinates (q, dq)
%   - params    : Struct with parameters
%
% Output:
%   - ddq          : Generalized accelerations for the joints (8x1) for the current simulation step
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
q = gc.q;      % Generalized coordinates (8x1)
dq = gc.dq;    % Generalized velocities (8x1)

M = M_fun_solution(q);
b = b_fun_solution(q,dq);
g = g_fun_solution(q);


%% Compute generalized acceleration of the system

% TODO: Implement ddq here
ddq = M\(tau -  b - g);

end