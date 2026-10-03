% Initialize a struct containing the body lengths in meters.
params = struct;

% Main parameters
params.l0 = 0.30;
params.l1 = 0.05;
params.l2 = 0.25;
params.l3 = 0.35;

% Initialize a random vector of joint positions.
q = rand(6,1);