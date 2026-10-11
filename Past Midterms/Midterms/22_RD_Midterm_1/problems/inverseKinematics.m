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
    q_iter = zeros(3,1);

end
