function [ Dq ] = kinematicStandupControl( q, p_hip_joint_des, params )  
  % Inputs:
  %  q               : current joint angles (6x1)
  %  p_hip_joint_des : desired hip pose in inertial frame (3x1)
  %  params          : a struct of parameters

  % Output:
  % Dq               : generalized coordinates velocity command (6x1)

  % Proportional controller gain
  K_p = 1;

  % Pseudo_inverse damping coefficient
  lambda = 1e-2;

  % Generalized coordinates
  q0 = q(1);
  q1 = q(2);
  q2 = q(3);
  q3 = q(4);
  q4 = q(5);
  q5 = q(6);
  l0 = params.l0;
  l1 = params.l1;

  % Implement your solution here...
  Dq = zeros(6,1);

end
