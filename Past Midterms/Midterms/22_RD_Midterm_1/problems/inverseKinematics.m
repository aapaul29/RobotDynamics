function [ q_iter ] = inverseKinematics( p_des, params )
% Inputs:
%  p_des         : desired gripper pose (3x1)
%  params        : a struct of parameters

% Output:
%  q_iter        : joint position command (3x1)

% Choose a pseudo_inverse damping coefficient
lambda = 1e-2;
% Choose a convergence threshold
epsilon = 1e-3;
% Maximum number of iterations
N_max = 100;

% Implement your solution here ...
% Initialize:
q_iter = [1; 1; 1];
N = 0;
p_curr = [jointTo2dGripperPosition_solution(q_iter, params);
    sum(q_iter(2:3) - pi / 2)];
p_err = norm(P_des - p_curr)


while (p_err > epsilon)
    N = N + 1;
    if N >= N_max;
        error('Max iterations reached');
    end
    % Compute Jacobian
    J_A = jointToGripperAnalyticalJacobian_solution(q_iter, params);
    % Compute pseudo Inverse
    J_Apsinv = pseudoInverseMat_solution(J_A, lambda);
    % Compute new q_iter
    q_iter = q_iter + J_Apsinv * (p_des - p_curr);
    % Update position
    p_curr = [jointTo2dGripperPosition_solution(q_iter, params);
        sum(q_iter(2:3) - pi / 2)];
    % Update error
    p_err = norm(p_des - p_curr);
end

end
