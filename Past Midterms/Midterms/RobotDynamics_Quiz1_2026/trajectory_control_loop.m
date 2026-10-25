% This script will allow you to see a visualization of the bicycle robot
% riding, lifting its front wheel and serving the drink
% (If visualization gets stuck, press Ctrl + C in the Command Window)

close all;
clear params;
init_workspace; % also initializes params

% Set to 0 to test your own solution, set to 1 to see how the solution should look like
use_solution = 0;

% Question 5 (Bonus): index of a stuck joint in q (0 = no stuck joint,
% 1 = q0 rear wheel, 2 = q1 neck, 3 = q2 head, 4 = q3 fork)
stuck_idx = 0;

% Question 5 (Bonus): set to 1 to use wheelieControlStuck instead of wheelieControl
use_stuck_controller = 0;

%% Control Loop
track_wheelie_trajectory(use_solution, params, stuck_idx, use_stuck_controller);
