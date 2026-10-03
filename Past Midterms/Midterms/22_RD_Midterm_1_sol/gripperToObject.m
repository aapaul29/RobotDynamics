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

    % inertia to base frame
    p_IB_I = [0; q0; l02];
    C_IB = eye(3);
    T_IB = [C_IB p_IB_I;
            zeros(1,3), 1];

    % base frame to sensor frame
    p_BS_B = [0; l01 + l03 + l04; - l04 * cot(alpha)];
    C_BS = [1, 0, 0;
            0, cos(- pi + alpha), -sin(- pi + alpha);
            0, sin(- pi + alpha), cos(- pi + alpha)];
    T_BS = [C_BS p_BS_B;
            zeros(1,3), 1];

    % sensor frame to object frame
    p_SO_S = [0; pO_y; pO_z];
    C_SO = [1, 0, 0;
            0, cos(pO_psi), -sin(pO_psi);
            0, sin(pO_psi), cos(pO_psi)];
    T_SO = [C_SO p_SO_S;
            zeros(1,3), 1];

    % object frame to grasp frame
    p_OG_O = [0; 0; 0];
    C_OG = [1, 0, 0;
            0, -1, 0;
            0, 0, -1];
    T_OG = [C_OG p_OG_O;
            zeros(1,3), 1];
    
    % inertia frame to grasp frame
    T_IG = T_IB * T_BS * T_SO * T_OG;

    % convert into minimal represetation
    p_des = zeros(3, 1);
    p_des(1:2) = T_IG(2:3, 4);
    p_des(3) = atan2(T_IG(3, 2), T_IG(2, 2));
    
end