function [ Dq ] = kinematicTrajectoryControl( q,  p_8_des, p_9_des, params )  
  % Inputs:
  %  q             : current joint angles (8x1)
  %  p_8_des       : desired 2D pose (y,z,Φ) for left EE (3x1)
  %  p_9_des       : desired 2D pose (y,z,Φ) for right EE (3x1)
  %  params        : a struct of parameters

  % Output:
  % Dq             : generalized coordinates velocity command (8x1)

  % Proportional controller gain. Do NOT modify this.
  K_p = 10;

  % Pseudo_inverse damping coefficient
  lambda = 1e-2;

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
  % Access current position
  [p_I8_I, p_I9_I] = jointTo2DPose_solution(q, params);
  r_curr = [p_I8_I; p_I9_I];
  % Access Jacobians
  [J_I8_2d, J_I9_2d] = jointTo2DJacobian_solution(q, params);
  J = [J_I8_2d; J_I9_2d];
  % pseudo Invers
  J_psinv = pseudoInverseMat_solution(J, lambda);
  % compute error
  r_err = [p_8_des; p_9_des] - r_curr;

  
  Dq = J_psinv * K_p * r_err; % This is just a placeholder.
    
end