% Initialize a struct containing the body lengths in meters.
params = struct;

% Main parameters
params.l0 = 4.5;
params.l1 = 8.22;
params.l2 = 0.27;
params.l3 = 0.25;
params.l4 = 1.5;
params.l5 = 1.5;
params.l6 = 0.3;
params.l7 = 0.5;

% Initialize a random vector of joint positions.
q = rand(4,1);
