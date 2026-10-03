function [ J_IG_r ] = jointToRotationJacobian(q, params)
  % Inputs:  
  %     q: a 3x1 vector of generalized coordinates
  %     params: a struct of parameters
  %
  % Outputs:
  %     J_IG_r: rotation Jacobian of gripper in inertia frame (3,3)

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
    
  % Implement your solution here...
  
  % joint vectors
  n1_I = [1; 0; 0];
  n2_I = [1; 0; 0];
  
  % rotation Jacobian
  J_IG_r = [zeros(3,1), n1_I, n2_I];

end