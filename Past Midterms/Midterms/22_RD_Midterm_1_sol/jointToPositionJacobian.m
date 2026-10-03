function [ J_IG_p ] = jointToPositionJacobian(q, params)
  % Inputs:  
  %     q: a 3x1 vector of generalized coordinates
  %     params: a struct of parameters
  %
  % Outputs:
  %     J_IG_p: position Jacobian of gripper in inertia frame (3,3)
  
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
  
  % Note: Formula in the script, eq. 2.155, is only applicable 
  %     for rotating joints, which are on the arm. For the base that
  %     only translates, we can account for its component directly.
  %
  % We apply eq. 2.154: J_IG_I = J_IB_I + J_BG_I
  %
  % Since, B is not a rotating frame, J_BG_B = J_BG_I

  % joint vectors (arm)
  n1_B = [1; 0; 0];
  n2_B = [1; 0; 0];
  
  % compute position vector from P1 to P2 in frame B
  p_12_1 = [0; 0; l11];
  C_B1 = [1, 0, 0;
          0, cos(q1), -sin(q1);
          0, sin(q1), cos(q1)];   
  p_12_B = C_B1 * p_12_1;

  % compute position vector from P2 to G in frame B
  p_2G_2 = [0; l22; l21];
  C_12 = [1, 0, 0;
          0, cos(q2), -sin(q2);
          0, sin(q2), cos(q2)];
  C_B2 = C_B1 * C_12;
  p_2G_B = C_B2 * p_2G_2;
  
  % compute position vector from P1 to G in frame B
  p_1G_B = p_12_B + p_2G_B;
  
  % linear Jacobian (from base to gripper)
  J_BG_p = [zeros(3, 1), cross(n1_B, p_1G_B), cross(n2_B, p_2G_B)];
  % linear Jacobian (from inertia to base)
  J_IB_p = [[0; 1; 0], zeros(3, 1), zeros(3, 1)];

  % linear Jacobian (from inertia to gripper)
  J_IG_p = J_IB_p + J_BG_p;

end