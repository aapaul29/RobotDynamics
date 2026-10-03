% Initialize the workspace.
init_workspace;
init_params;

%% Exercise 1.
disp('Running exercise 1...');
try
  T_IE = jointToGripperPose(q, params);
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
  J_IE_p = jointToPositionJacobian(q, params);
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
  q = [0.6562; -0.0552; -0.7973; -0.5437];
  w_IE_des_I = [1.5; 0.6; pi/10];
  q_dot_des = gripperToJointPrioritized(w_IE_des_I, q, params);
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
  q = [0.3481; 0.7670; -0.9997; -1.1635];
  p_IO_des_I = [8.3238; 0.9435; 100 * pi / 180];
  w_IO_des_I = [1.3792; 1.3501; 0];
  q_dot_cmd = kinematicTrajectoryControl(q, p_IO_des_I, w_IO_des_I, params);
  disp('done.');
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end

