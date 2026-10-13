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

  T_I8 = [];
  
end