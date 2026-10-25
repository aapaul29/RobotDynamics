function [ J_ID_r ] = jointToRotationJacobian(q, params)
  % Inputs:
  %     q: a 4x1 vector of generalized coordinates
  %     params: a struct of parameters
  %
  % Outputs:
  %     J_ID_r: rotation Jacobian of drink frame {D} wrt inertial
  %             frame {I} expressed in inertial frame {I} (3x4)

  % link lengths (meters)
  r     = params.r;
  lNy   = params.lNy;
  lNz   = params.lNz;
  alpha = params.alpha;
  l1    = params.l1;
  lDy   = params.lDy;
  lDz   = params.lDz;

  % Generalized coordinates
  q0 = q(1);  % rear wheel (prismatic, along y_I)
  q1 = q(2);  % neck joint
  q2 = q(3);  % head joint
  q3 = q(4);  % fork (prismatic, along the fork axis)

  % Implement your solution here...

  J_ID_r = [];

end
