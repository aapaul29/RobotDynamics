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
% Feedback gain on the pose error
K = 20;

% --- Step 1: object target -> gripper target ---
% phi is measured from z_I, rotating about y_I: z_E in (x_I, z_I) = [sin(phi); cos(phi)]
% (q1 + q2 + q3 is measured from z_0 = -z_I, hence phi = q1 + q2 + q3 + pi)
phi_des = p_IO_des_I(3);
z_E_des = [sin(phi_des); cos(phi_des)];

% O is offset by l7 along z_E, so walk back from O to E
r_IE_des = p_IO_des_I(1:2) - l7 * z_E_des;
p_IE_des = [r_IE_des; phi_des];

% Rigid grasp with zero desired angular velocity -> gripper twist = object twist
w_IE_des = w_IO_des_I;

% --- Step 2: track the gripper target ---
% Current gripper pose
r_IE = jointTo2DGripperPosition_solution(q, params);
phi = q1 + q2 + q3 + pi;
p_IE = [r_IE(:); phi];

% Jacobian and its damped pseudo-inverse
J = jointTo2DGripperJacobian_solution(q, params);
J_pinv = pseudoInverseMat_solution(J, lambda);

% Feedforward twist + proportional feedback on the pose error
q_dot_cmd = J_pinv * (w_IE_des + K * (p_IE_des - p_IE));

end
