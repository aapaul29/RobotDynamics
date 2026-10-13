% Initialize a struct containing the body lengths in meters.
params = struct;

% Main parameters
params.lI = 0.02;
params.lW = 0.025;
params.lB = 0.05;
params.l1 = 0.03;
params.l2 = 0.03;
params.l3 = 0.01;

% Initialize a random vector of joint positions.
q = rand(8,1);