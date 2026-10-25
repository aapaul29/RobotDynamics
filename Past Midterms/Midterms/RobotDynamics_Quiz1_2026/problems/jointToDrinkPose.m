function [ T_ID ] = jointToDrinkPose(q, params)
  % Inputs:
  %     q: a 4x1 vector of generalized coordinates.
  %     params: a struct of parameters.
  %
  % Outputs:
  %     T_ID: Homogenous transform from drink frame {D} to inertial
  %           frame {I}

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

  T_ID = [];

end
