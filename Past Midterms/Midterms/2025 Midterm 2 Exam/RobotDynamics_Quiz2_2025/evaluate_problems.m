% Initialize the workspace.
init_workspace

%% Setup
% generalized coordinates
gc = generate_gc();
q_eval = [1.0, 0.5, 0.0, 1.5, 1.2, -0.1, 0.6, 0.2]';
dq_eval = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]';

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
    tau_eval = ones(8, 1)*0.0001;
    ddq = Q2_forward_dynamics(gc_eval, tau_eval, params);

    disp('Done.');
    if ~any(ddq(:))
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
    I_r_ITd = zeros(3,1);
    I_v_Td = I_r_ITd;
    C_ITd = eye(3);

    user_tau = Q4_inverse_dynamics_control(params, gc_eval, kin, I_r_ITd, I_v_Td, C_ITd , 0.0);

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


