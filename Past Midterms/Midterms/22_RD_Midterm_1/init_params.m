% Initialize a struct containing the body lengths in meters.
params = struct;

% Main parameters
params.l01 = 0.34;
params.l02 = 0.22;
params.l03 = 0.15;
params.l04 = 0.1;
params.l11 = 0.7;
params.l21 = 0.85;
params.l22 = 0.1;
params.alpha = pi / 3;

% Initialize a random vector of joint positions.
q = rand(3,1);
