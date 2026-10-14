function [ T_I8 ] = jointToEEPose(q, params)
  % Inputs:
  %     q: a 8x1 vector of generalized coordinates (8, 1).
  %     params: a struct of parameters.
  %
  % Outputs:
  %     T_I8: Homogenous transform from left arm EE frame {P8} to inertial
  %           frame {I}

  % link lengths (meters)
  lI = params.lI;
  lW = params.lW;
  lB = params.lB;
  l1 = params.l1;
  l2 = params.l2;
  l3 = params.l3;

  % Generalized coordinates
  q0 = q(1);
  q1 = q(2);
  q2 = q(3);
  q3 = q(4);
  q4 = q(5);
  q5 = q(6);
  q6 = q(7);
  q7 = q(8);
    
  % Implement your solution here ...
  % Compute T_IB
  C_IB = eye(3);
  r_IB = [0; q0; lI + q1];
  T_IB = [C_IB, r_IB;
          zeros(1, 3), 1];
  % Compute T_B2 
  C_B2 = [1, 0, 0;
          0, cos(q2), -sin(q2);
          0, sin(q2), cos(q2)];
  r_B2 = [0; lW; lB];
  T_B2 = [C_B2, r_B2;
          zeros(1, 3), 1];
  % Compute T_24
  C_24 = [1, 0, 0;
          0, cos(q4), -sin(q4);
          0, sin(q4), cos(q4)];
  r_24 = [0; 0; l1];
  T_24 = [C_24, r_24;
          zeros(1, 3), 1];
  % Copute T_46
  C_46 = [1, 0, 0;
          0, cos(q6), -sin(q6);
          0, sin(q6), cos(q6)];
  r_46 = [0; 0; l2];
  T_46 = [C_46, r_46;
          zeros(1, 3), 1];
  % Compute T_68
  C_68 = eye(3);
  r_68 = [0; 0; l3];
  T_68 = [C_68, r_68;
          zeros(1, 3), 1];

  % Compute T_I8
  T_I8 =T_IB * T_B2 * T_24 * T_46 * T_68;
  
end