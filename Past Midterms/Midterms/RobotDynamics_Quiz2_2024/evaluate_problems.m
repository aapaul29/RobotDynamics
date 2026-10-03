% Initialize the workspace.
init_workspace

%% Setup
% generalized coordinates
gc = generate_gc();
q_eval = [1.0, 0.5, 0.0, 1.5, 1.2, -0.1]';
dq_eval = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]';

% Initialize the parameters for the mid-term exam.
params = init_params();

% Forward Kinematics
kin = generate_kin(gc.q, params);

% Forward Differential Kinematics
jac = generate_jac(gc, kin, params);


%% Q1.
disp('==============');
disp('Running Q1 ...');

try
  eom = Q1_generate_eom(gc, kin, params, jac);
  disp('eom generated.');
  
  disp('Running eom.M ...');
  M = eval(subs(eom.M, gc.q, q_eval));
  disp('Running eom.g ...');
  g = eval(subs(eom.g, gc.q, q_eval));
  disp('Running eom.b ...');
  b = eval(subs(eom.b, [gc.q, gc.dq], [q_eval, dq_eval]));
  
  disp('Done.');

  if ~any(M(:)) || ~any(b(:)) || ~any(g(:))
    warning('Some of the returned matrices are empty')
  end
  
catch ME
  warning('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end

%% Q2.
disp('==============');
disp('Running Q2 ...');

try
  gc_eval.q = q_eval;
  gc_eval.dq = dq_eval;
  tau_eval = ones(6, 1);
  [user_ddq_arm, user_q_base_next] = Q2_forward_dynamics(gc_eval, tau_eval, params);
  
  disp('Done.');
  if ~any(user_ddq_arm(:))
    warning('Returned accelerations are null')
  end

catch ME
  warning('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end


%% Q3.
disp('==============');
disp('Running Q3 ...');

try
  gc_eval.q = q_eval;
  gc_eval.dq = dq_eval;
  q_des = q_eval / 2.0;
  dq_des = dq_eval / 2.0;
  user_tau = Q3_gravity_compensation(params, gc_eval, q_des, dq_des);
  
  disp('Done.');

  if ~any(user_tau(:))
    warning('Returned torques are null')
  end

catch ME
  warning('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end


%% Q4.
disp('==============');
disp('Running Q4 ...');

try
  gc_eval.q = q_eval;
  gc_eval.dq = dq_eval;
  q_des = q_eval / 2.0;
  dq_des = dq_eval / 2.0;
  T_ITd = eval(subs(kin.T_IT, {'q0' 'q1' 'q2' 'q3' 'q4' 'q5'}, {q_des(1) q_des(2) q_des(3) q_des(4) q_des(5) q_des(6)}));
  I_r_ITd = T_ITd(1:3, 4);
  I_v_Td = jac.I_Jp_T * dq_des;
  I_v_Td = eval(subs(I_v_Td, {'q0' 'q1' 'q2' 'q3' 'q4' 'q5'}, {q_des(1) q_des(2) q_des(3) q_des(4) q_des(5) q_des(6)}));
  C_ITd = eye(3);
  
  user_tau = Q4_inverse_dynamics_control(params, gc_eval, kin, I_r_ITd, C_ITd, I_v_Td , 0.0);

  disp('Done.');
    
  if ~any(user_tau(:))
    warning('Returned torques are null')
  end

catch ME
  warning('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end


