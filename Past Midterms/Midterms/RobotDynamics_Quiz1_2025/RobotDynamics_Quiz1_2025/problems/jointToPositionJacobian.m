function [ J_I8_p ] = jointToPositionJacobian(q, params)
  % Inputs:  
  %     q: a 8x1 vector of generalized coordinates (8, 1).
  %     params: a struct of parameters
  %
  % Outputs:
  %     J_I8_p: position Jacobian of left arm EE frame {P8} wrt inertia 
  %             frame {I} expressed in inertial frame {I}
  
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
  
  % Implement your solution here...
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
  % Normal vectors
  n0 = [0; 1; 0];
  n1 = [0; 0; 1];
  n2 = [1; 0; 0];
  n3 = [1; 0; 0];
  n4 = [1; 0; 0];
  n5 = [1; 0; 0];
  n6 = [1; 0; 0];
  n7 = [1; 0; 0];
  
  %joint 2
  T_28 = T_24 * T_46 * T_68;
  C_I2 = C_IB * C_B2;
  r_28  = T_28(1:3, 4);
  I_r_28 = C_I2 * r_28;

  % joint 4
  T_48 = T_46 * T_68;
  C_I4 = C_I2 * C_24;
  r_48 = T_48(1:3, 4);
  I_r_48 = C_I4 * r_48;

  % joint 6
  C_I6 = C_I4 * C_46;
  I_r_68 = C_I6 * r_68;
    



  J_I8_p = [n0, n1, cross(n2, I_r_28), zeros(3, 1), cross(n4, I_r_48), zeros(3, 1), cross(n6, I_r_68), zeros(3, 1)];
  
end

