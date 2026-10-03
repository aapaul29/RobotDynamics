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

% Compute the current base Jacobian
J_base = jointToBaseJacobian_solution(q, params);  % Base Jacobian (3x6)

% Compute the current tray Jacobian
J_tray = jointToTrayJacobian_solution(q, params);  % Tray Jacobian (3x6)

% Desired base velocities (tracking the desired pose)
% Desired base pose: [Ix_IB, Iy_IB, Iphi_IB]
desired_base_vel = K_p * (p_base_des - q(1:3));

% Set desired tray velocities to zero (keeping the tray stationary)
desired_tray_vel = [0; 0; 0];  % Zero linear and angular velocity at the tray

% Concatenate desired velocities
desired_vel = [desired_base_vel; desired_tray_vel];

% Construct the augmented Jacobian to minimize both base and tray movement
J_augmented = [J_base; J_tray];  % Augmented Jacobian (6x6)

% Compute joint velocities using the damped pseudo-inverse
Dq = pseudoInverseMat_solution(J_augmented, lambda) * desired_vel;

end
