%% Initialize parameters
init_params;

%% Generate oscillating motion
timesteps = 200;
t = linspace(0, 2*pi, timesteps);

% Base motion
q0 = 0.05 * sin(t);           % horizontal shift (y-axis)
q1 = 0.02 * sin(t) + 0.05;    % vertical lift between 0.05 and 0.15

% Left arm (shoulder, elbow, wrist)
amp = pi/8;
q3 = amp * sin(t) + pi/2;           % left shoulder
q5 = amp * sin(t + pi/3);    % left elbow
q7 = amp * sin(t - 2*pi/3) - pi / 2;  % left wrist

% Right arm is the mirror (negative of left)
q2 = -q3;                    % right shoulder
q4 = -q5;                    % right elbow
q6 = -q7;                    % right wrist

% Combine all into full trajectory
Q = [q0; q1; q2; q3; q4; q5; q6; q7];

% Time step for animation
t_s = 0.01;

%% Run animation
animate_robot(Q, [], t_s, params);