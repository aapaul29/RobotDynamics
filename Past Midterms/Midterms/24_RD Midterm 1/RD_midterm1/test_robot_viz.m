%% Load params
init_params;

%% Run test visualization
q = [0 0.4 0.4 1.0 1.0 pi/2 - 2.4]; % x position, y position, base angle, 3 x arm joints
q_ref = [0, 0, 0]; % x position, y position, orientation

timesteps = 40;
Q = repmat(q', 1, timesteps);
theta = linspace(0, 2*pi, timesteps); % Create an array of angles
x = 0.5 * cos(theta) + 1.0; % X coordinates of the circle
y = 0.5 * sin(theta) + 1.0; % Y coordinates of the circle

movement = (1:1:timesteps) * 0.02; % move some joints
Q(1, :) = Q(1, :) + movement;
Q(3, :) = Q(3, :) - movement;
Q(6, :) = Q(6, :) + movement;

t_s = 0.001; % as fast as possible
Q = [Q, fliplr(Q), Q, fliplr(Q)]; % back and forth and back and forth...

animate_robot(Q, [], t_s, params);