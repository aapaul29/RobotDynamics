function [p_des] = gripperToObject( q, pO, params )
    % Inputs:
    %  q            : current joint angles (3x1)
    %  pO           : detected object pose in sensor frame (3x1)
    %  params       : a struct of parameters

    % Output:
    %  p_des        : desired gripper pose in inertia frame (3x1)

    % link lengths (meters)
    l01 = params.l01;
    l02 = params.l02;
    l03 = params.l03;
    l04 = params.l04;
    l11 = params.l11;
    l21 = params.l21;
    l22 = params.l22;
    % angle (radians)
    alpha = params.alpha;

    % Joint positions
    q0 = q(1);
    q1 = q(2);
    q2 = q(3);
    
    % Object pose (in sensor frame)
    pO_y = pO(1);
    pO_z = pO(2);
    pO_psi = pO(3);
    
    % Implement your solution here ...
    
end
