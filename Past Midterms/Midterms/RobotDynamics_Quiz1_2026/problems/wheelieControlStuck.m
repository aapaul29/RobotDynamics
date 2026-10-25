function [ Dq ] = wheelieControlStuck( q, x_des, z_D_des, stuck_idx, params )
  % Inputs:
  %  q             : current generalized coordinates (4x1)
  %  x_des         : desired primary task coordinates [y_B; phi_D; z_F] (3x1)
  %                  y_B   : position of the rear axle along y_I
  %                  phi_D : tilt of the drink frame {D} about x_I
  %                  z_F   : height of the front axle {F}
  %  z_D_des       : desired drink height (secondary task) (1x1)
  %  stuck_idx     : index of the stuck joint in q
  %                  (1 = q0 rear wheel, 2 = q1 neck, 3 = q2 head, 4 = q3 fork)
  %  params        : a struct of parameters

  % Output:
  % Dq             : generalized coordinates velocity command (4x1),
  %                  Dq(stuck_idx) must be zero

  % Proportional controller gain. Do NOT modify this.
  K_p = 10;

  % Pseudo_inverse damping coefficients
  lambda   = 1e-2;  % primary task
  lambda_2 = 1e-1;  % secondary task (drink height)

  % Implement your solution here...

  Dq = 0 * q; % This is just a placeholder.

end
