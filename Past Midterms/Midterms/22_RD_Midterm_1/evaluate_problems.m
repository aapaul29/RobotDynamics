% Initialize the workspace.
init_workspace;
init_params;

%% Exercise 1.
disp('Running exercise 1...');
try
  T_IS = jointToGripperPose(q, params);
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
  p_des = [1.0; 1.0; -pi / 4];
  q_des = inverseKinematics(p_des, params);
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
  I_p_SO = [-0.5; 1.0; pi - params.alpha];
  I_p_IG = gripperToObject(q, I_p_SO, params);
  disp('done.');
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end

