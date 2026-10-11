function [ q_dot_cmd ] = kinematicTrajectoryControl( q, p_IO_des_I, w_IO_des_I, params )
% Inputs:
%  q             : 4x1 current joint angles
%  p_des         : 3x1 desired 2D object pose
%  w_des         : 3x1 desired 2D object twist
%  params        : a struct of parameters

% Output:
%  q_dot_cmd     : 4x1 joint velocities command

% link lengths (meters)
l0 = params.l0;
l1 = params.l1;
l2 = params.l2;
l3 = params.l3;
l4 = params.l4;
l5 = params.l5;
l6 = params.l6;
l7 = params.l7;

% Joint positions
q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);

% Pseudo-inverse damping coefficient
lambda = 1e-3;

% Implement your solution here...
q_dot_cmd = zeros(4,1);

end
