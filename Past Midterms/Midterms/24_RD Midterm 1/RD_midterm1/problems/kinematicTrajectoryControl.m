function [ Dq ] = kinematicTrajectoryControl( q, p_base_des, params )
% Inputs:
%  q             : current joint angles (6x1)
%  p_base_des    : desired 2D base pose (x,y,q1) (3x1)
%  params        : a struct of parameters
%
% Output:
% Dq             : generalized coordinates velocity command (6x1)

% Proportional controller gain
K_p = 10;

% Pseudo_inverse damping coefficient
lambda = 1e-2;

% Implement your solution here...
Dq = zeros(6,1);

end
