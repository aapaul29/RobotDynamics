% Initialize the workspace.
init_workspace; % also initializes params

%% Exercise 1.
disp('Running exercise 1...');
try
  T_I8 = jointToEEPose(q, params);
  disp('done.');
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end

%% Exercise 2.
disp('Running exercise 2...');
try
  Jp = jointToPositionJacobian(q, params);
  disp('done.');
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end

%% Exercise 3.
disp('Running exercise 3...');
try
  Jr = jointToRotationJacobian(q, params);
  disp('done.');
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end

%% Exercise 4.
disp('Running exercise 4...');
try
  q = [0.2; 0.02; 0.10; 0.20; 0.30; -0.10; -0.20; -0.30];
  p_8_des = [0; 0.16; 0.15];
  p_9_des = [0; 0.24; 0.15];
  trajectory_Dq = kinematicTrajectoryControl(q, p_8_des, p_9_des, params);
  disp('done.');
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end
