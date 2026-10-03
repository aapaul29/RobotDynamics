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
   % % Step 1: Compute the base Jacobian (for the hip frame task)
     J_base = jointToBaseJacobian_solution(q, params);  % 3x6
   % 
   %  % Step 2: Compute the tray Jacobian (to minimize tray velocity)
     J_tray = jointToTrayJacobian_solution(q, params);  % 3x6
     J_tray = J_tray(end,:); % 1x6
   % 
   %  % Step 3: Compute the desired hip velocities (to stand the robot up)
   %  % Assuming desired_hip_pose = [Ix_IH, Iy_IH, Iphi_IH]'
     p_base_des = p_hip_joint_des-[l1;-l0;0];
   %  %p_hip_joint_des = p_hip_joint_des-[l0;l1;0]/2; % now is a base frame velocity
   %  % and q(1:3) represents the current base pose.
     desired_hip_base = K_p * (p_base_des - q(1:3));  % Proportional control for hip pose
   % 
   %  % Step 4: Set the desired tray angular velocity to zero (to keep the tray level)
     desired_tray_vel = 0;  % Zero linear and angular velocity at tray
   % 
   %  % Step 5: Concatenate the velocities for the augmented Jacobian
   %  desired_vel = [desired_hip_vel; desired_tray_vel];
   % 
   %  % Step 6: Construct the augmented Jacobian
   %  % Combine both the base and tray Jacobians
   %  J_augmented = [J_base; J_tray];  % 6x6 Jacobian
   % 
   %  % Step 7: Use the damped pseudo-inverse to compute joint velocities (q_dot)
   %  q_dot = pseudoInverseMat_solution(J_augmented, lambda) * desired_vel;
    q1_dot = pseudoInverseMat_solution(J_tray, lambda)*desired_tray_vel;
    N1 = null(J_tray);
    temp = J_base*N1;
    q2_dot = pseudoInverseMat_solution(temp, lambda)*(desired_hip_base);
    

   q_dot = q1_dot+N1*q2_dot;

Dq =q_dot;

end