% Checks that your functions run and that their outputs have the right size.
% This script does NOT test if your solutions are correct.

% Initialize the workspace.
init_workspace; % also initializes params

%% Exercise 1.
disp('Running exercise 1...');
try
  q = [0.4; 0.5; -0.6; 0.3];
  T_ID = jointToDrinkPose(q, params);
  if checkOutput('T_ID', T_ID, [4, 4]), disp('done.'); end
catch ME
  printError(ME);
end

%% Exercise 2.
disp('Running exercise 2...');
try
  J_ID_p = jointToPositionJacobian(q, params);
  if checkOutput('J_ID_p', J_ID_p, [3, 4]), disp('done.'); end
catch ME
  printError(ME);
end

%% Exercise 3.
disp('Running exercise 3...');
try
  J_ID_r = jointToRotationJacobian(q, params);
  if checkOutput('J_ID_r', J_ID_r, [3, 4]), disp('done.'); end
catch ME
  printError(ME);
end

%% Exercise 4.
disp('Running exercise 4...');
try
  q = [0; 0.8; -0.8; 0.37];
  x_des = [0.05; 0; 0.2];
  z_D_des = 1.0;
  Dq = wheelieControl(q, x_des, z_D_des, params);
  if checkOutput('Dq', Dq, [4, 1], true), disp('done.'); end
catch ME
  printError(ME);
end

%% Exercise 5a (Bonus).
disp('Running exercise 5a...');
try
  achievable = achievableTasks(2);
  if checkOutput('achievable', achievable, [4, 1])
    if all(achievable(:) == 0 | achievable(:) == 1)
      disp('done.');
    else
      disp('  achievable should only contain true/false (1/0) values.');
    end
  end
catch ME
  printError(ME);
end

%% Exercise 5b (Bonus).
disp('Running exercise 5b...');
try
  q = [0; 0.8; -0.8; 0.37];
  x_des = [0.05; 0; 0.2];
  z_D_des = 1.0;
  stuck_idx = 2;
  Dq = wheelieControlStuck(q, x_des, z_D_des, stuck_idx, params);
  if checkOutput('Dq', Dq, [4, 1], true)
    if Dq(stuck_idx) ~= 0
      disp('  Dq(stuck_idx) has to be zero.');
    else
      disp('done.');
    end
  end
catch ME
  printError(ME);
end

% ------------------ helper: check the size of an output ------------------
function ok = checkOutput(name, value, expected_size, check_zero)
  if nargin < 4, check_zero = false; end
  ok = false;
  if isempty(value)
    fprintf('  %s is empty. Not implemented yet?\n', name);
  elseif ~(isnumeric(value) || islogical(value))
    fprintf('  %s has to be a numeric array.\n', name);
  elseif any(size(value) ~= expected_size)
    fprintf('  %s should be of size %dx%d, but is %dx%d.\n', name, expected_size, size(value));
  elseif any(~isfinite(value(:)))
    fprintf('  %s contains NaN or Inf values.\n', name);
  elseif check_zero && all(value(:) == 0)
    fprintf('  %s is all zeros. Not implemented yet?\n', name);
  else
    ok = true;
  end
end

% ------------------ helper: print an error ------------------
function printError(ME)
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end
