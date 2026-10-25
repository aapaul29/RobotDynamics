function [ Dq ] = wheelieControl( q, x_des, z_D_des, params )
  % Inputs:
  %  q             : current generalized coordinates (4x1)
  %  x_des         : desired primary task coordinates [y_B; phi_D; z_F] (3x1)
  %                  y_B   : position of the rear axle along y_I
  %                  phi_D : tilt of the drink frame {D} about x_I
  %                  z_F   : height of the front axle {F}
  %  z_D_des       : desired height of the drink frame {D} (secondary task) (1x1)
  %  params        : a struct of parameters

  % Output:
  % Dq             : generalized coordinates velocity command (4x1)

  % Proportional controller gain. Do NOT modify this.
  K_p = 10;

  % Pseudo_inverse damping coefficients
  lambda   = 1e-2;  % primary task
  lambda_2 = 1e-1;  % secondary task (drink height)

  % Implement your solution here...

  Dq = 0 * q; % This is just a placeholder.

end
