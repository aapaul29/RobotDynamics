% Initialize the workspace.
init_workspace; % also initializes params

%% Exercise 1.
disp('Running exercise 1...');
try
  T_IT = jointToTrayPose(q, params);
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
  q = [0; 0.50; 0.10; -4.08; 1.26; 4.30];
  p_base_des = [0; 0.54; 0.10];
  entertain_Dq = kinematicTrajectoryControl(q, p_base_des, params);
  disp('done.');
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end

%% Exercise 5.
disp('Running exercise 5...');
try
  q = [0; 0.50; 0.10; -5.34; 1.26; 5.55];
  p_hip_joint_des = [0; 0.50; 1.57];
  entertain_Dq = kinematicStandupControl(q, p_hip_joint_des, params);
  disp('done.');
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end