% Initialize a struct containing the body lengths in meters.
% Planar bicycle robot (inspired by the RAI UMV) carrying a drink on its Head.
params = struct;

% Main parameters
params.r     = 0.15;  % wheel radius (rear and front)
params.lNy   = 0.43;  % neck joint {N} (front of the bike frame) from rear axle along y_B
params.lNz   = 0.32;  % neck joint {N} (front of the bike frame) from rear axle along z_B
params.alpha = pi/6;  % fork angle, measured from -z_B towards +y_B
params.l1    = 0.40;  % neck: {N} to head joint {H}
params.lDy   = 0.18;  % drink frame {D} from head joint {H} along y_H (forward)
params.lDz   = 0.17;  % drink frame {D} from head joint {H} along z_H (up)

% Initialize a random vector of generalized coordinates.
q = rand(4,1);
