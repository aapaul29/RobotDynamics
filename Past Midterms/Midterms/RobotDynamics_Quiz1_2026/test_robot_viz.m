%% Initialize parameters
init_params;

%% Generate a motion through all joints
timesteps = 300;
t = linspace(0, 2*pi, timesteps);

q0 = 0.6 * t / (2*pi);                              % ride forward [m]
q1 = 0.5 + 0.3 * sin(t + pi/3);                     % neck
q2 = -q1;                                           % keep the head level
q3 = params.lNz/cos(params.alpha) - 0.12*max(sin(t), 0);  % retract fork, lift front wheel

Q = [q0; q1; q2; q3];

% Time step for animation
t_s = 0.01;

%% Run animation
animate_robot(Q, [], t_s, params);
