function [ J_I8_r ] = jointToRotationJacobian(q, params)
  % Inputs:  
  %     q: a 8x1 vector of generalized coordinates
  %     params: a struct of parameters
  %
  % Outputs:
  %     J_I8_r: rotation Jacobian of left arm EE frame {P8} wrt inertia 
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
  % Normal vectors
  n0 = [0; 1; 0];
  n1 = [0; 0; 1];
  n2 = [1; 0; 0];
  n3 = [1; 0; 0];
  n4 = [1; 0; 0];
  n5 = [1; 0; 0];
  n6 = [1; 0; 0];
  n7 = [1; 0; 0];

  J_I8_r = [zeros(3, 1), zeros(3, 1), n2, zeros(3, 1), n4, zeros(3, 1), n6, zeros(3, 1)];
  
end

