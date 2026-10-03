function [ T_IG ] = jointToGripperPose( q, params )
  % Inputs:
  %     q: a 3x1 vector of generalized coordinates (3, 1).
  %     params: a struct of parameters.
  %
  % Outputs:
  %     T_IG: Homogenous transform from inertia to gripper frame (4, 4)

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
    
  % Implement your solution here ...

  % inertia to base frame
  p_IB_I = [0; q0; l02];
  C_IB = eye(3);
  T_IB = [C_IB p_IB_I;
          zeros(1,3), 1];

  % base frame to link 1
  p_B1_B = [0; l01; 0];
  C_B1 = [1, 0, 0;
          0, cos(q1), -sin(q1);
          0, sin(q1), cos(q1)];
  T_B1 = [C_B1 p_B1_B;
          zeros(1,3), 1];
  
  % link 1 to link 2
  p_12_1 = [0;0; l11];
  C_12 = [1, 0, 0;
          0, cos(q2), -sin(q2);
          0, sin(q2), cos(q2)];
  T_12 = [C_12 p_12_1;
          zeros(1,3), 1];
  
  % link 2 to gripper
  p_2G_2 = [0; l22; l21];
  C_2G = [1, 0, 0;
          0, 0, 1;
          0, -1, 0];
  T_2G = [C_2G, p_2G_2;
          zeros(1,3), 1];
  
  % final: inertia to gripper       
  T_IG = T_IB * T_B1 * T_12 * T_2G;

end