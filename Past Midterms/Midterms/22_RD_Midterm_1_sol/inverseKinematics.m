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
    % initialize the IK
    q_iter = [1; 1; 1];
    
    % Implement your solution here...

    N = 0;
    % current gripper pose
    p_curr = [jointTo2DGripperPosition_solution(q_iter, params); sum(q_iter(2:3)) - pi / 2];
    % computer error
    p_error = norm(p_des - p_curr);

    % iteratively find solution
    while p_error > epsilon
        % incremenet counter
        N = N + 1;
        if N >= N_max
           error('Maximum IK iterations reached...');
        end
        % analytic Jacobian
        Ja = jointToGripperAnalyticalJacobian_solution(q_iter, params);
        % pseudo-inverse of the analytic Jacobian
        Ja_pinv = pseudoInverseMat_solution(Ja, lambda);
        % incremental update of joint angles
        q_iter = q_iter + Ja_pinv * (p_des - p_curr);
        % current gripper pose
        p_curr = [jointTo2DGripperPosition_solution(q_iter, params); sum(q_iter(2:3)) - pi / 2];
        % computer error
        p_error = norm(p_des - p_curr);
    end
    
end